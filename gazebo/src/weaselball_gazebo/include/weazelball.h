#ifndef _WEAZELBALL_H_
#define _WEAZELBALL_H_

//-----------------------------------------------------------------------------

#include "common.h"
#include "state.h"
#include "vicon.h"
#include <vector>
#include <string>
#include <sstream>

#include <boost/shared_ptr.hpp>

#include <Ravelin/Quatd.h>
#include <Ravelin/AAngled.h>

/**
Instances of wb_shell_state_c are intended to contain the state of the 
weazelball shell at a given time.  The shell state is at least a 7 element tuple
that combines a 3-dimension position vector and a 4 dimensional rotation unit 
quaternion.  The shell state allocates an additional 6 elements in case velocity
data is calculated and retained for the state.
*/
//-----------------------------------------------------------------------------
class wb_shell_state_c;
//-----------------------------------------------------------------------------
typedef boost::shared_ptr<wb_shell_state_c> wb_shell_state_ptr;
//-----------------------------------------------------------------------------
class wb_shell_state_c : public state_c {
private:
public:
  double vicon_time;     //< time of this state in vicon reference frame
  double virtual_time;   //< time of this state in virtual reference frame
public:
  // constuctor to copy vicon state into this instance
  wb_shell_state_c( wb_vicon_state_ptr vicon_state ) : state_c(13) {
    assert( vicon_state );

    for( unsigned i = 0; i < vicon_state->size(); i++ ) {
      _x[i] = vicon_state->val(i);
    }
    for( unsigned i = vicon_state->size(); i < size(); i++ ) {
      _x[i] = 0.0;
    }
    _t = vicon_state->t();
  }

  virtual ~wb_shell_state_c( void ) {}

  void print( void ) {
    state_c::print( "wb_shell_state" );
  }
};

/**
wb_session_c is the set of all states for an entire trial run.
*/
//-----------------------------------------------------------------------------
class wb_session_c;
//-----------------------------------------------------------------------------
typedef boost::shared_ptr<wb_session_c> wb_session_ptr;
//-----------------------------------------------------------------------------
class wb_session_c {
private:
  unsigned _id;              //< the index of the session (aka trial_id)

public:
  wb_session_c( void ) {
    _id = 0;
  }
  virtual ~wb_session_c( void ) {}

  std::vector<wb_shell_state_ptr> shell_states;

  // Getters
  unsigned id( void ) { return _id; }

  // Checks the change in rotation between vicon frames.  Finds the mean 
  // rotation over all vicon frames and then finds all rotational differences
  // that lies three standard deviations from the mean and reports them.  These
  // cases can then be manually evaluated as to whether they need interpolation
  void evaluate( wb_vicon_session_ptr vicon_session ) {
    _id = vicon_session->id();

    double total_dtheta = 0.0;
    std::vector<double> dthetas;
    double virtual_time = 0.0;        // virtual time of the sim
    double adjusted_vicon_time = 0.0; // normalized vicon time 
    double initial_vicon_time = 0.0;  // to normalize vicon time
    std::vector<double> flips;

    for( unsigned i = 0; i < vicon_session->states.size(); i++ ) {
      wb_shell_state_ptr shell_state = wb_shell_state_ptr( new wb_shell_state_c( vicon_session->states[i] ) );

      if( i == 0 ) {
        initial_vicon_time = shell_state->t();
        shell_state->vicon_time = 0.0;
        shell_state->virtual_time = 0.0;
        shell_states.push_back( shell_state );
        continue;
      }

      adjusted_vicon_time = shell_state->t() - initial_vicon_time;
      shell_state->vicon_time = adjusted_vicon_time;

      wb_shell_state_ptr prev_state = shell_states[shell_states.size()-1];

      Ravelin::Quatd q0, q1;
      double dt;
      double vicon_dt;

      q0 = Ravelin::Quatd( prev_state->val(3), prev_state->val(4), prev_state->val(5), prev_state->val(6) );
      q0.normalize();
      q1 = Ravelin::Quatd( shell_state->val(3), shell_state->val(4), shell_state->val(5), shell_state->val(6) );
      q1.normalize();

      dt = 1.0 / VICON_SAMPLE_RATE;
      virtual_time += dt;
      shell_state->virtual_time = virtual_time;
/* 
      vicon_dt = shell_state->vicon_time - prev_state->vicon_time;
      if( fabs(vicon_dt - dt) > 0.5 * dt ) {
        printf( "possible dropped vicon frame at sample %d: vicon_dt[%f], virtual_t[%f], vicon_t[%f]\n", i, vicon_dt, virtual_time, shell_state->vicon_time );
      }
*/
      double dtheta = Ravelin::Quatd::calc_angle( q0, q1 ) / dt;
      total_dtheta += dtheta;
      // prints each of the states 
      //printf( "%d: dt[%f], dtheta[%f], total_dtheta[%f]\n", i, dt, dtheta, total_dtheta );
      shell_states.push_back( shell_state );
      dthetas.push_back( dtheta );
    }
    double sz = shell_states.size();
    double mean = total_dtheta / sz;
    //printf( "average_dtheta[%f]\n", mean );

    double variance = 0.0;
    for( unsigned i = 0; i < dthetas.size(); i++ ) {
      double v = dthetas[i] - mean;
      variance += v * v;
    }
    variance /= dthetas.size();
    double sigma = sqrt( variance );
    double sigma2 = sigma * 2.0;
    double sigma3 = sigma * 3.0;

    printf( "samples[%d], mean[%f], variance[%f], deviation[%f], 2 deviations[%f], 3 deviations[%f]\n", shell_states.size(), mean, variance, sigma, sigma2, sigma3 );

    unsigned ct = 0;
    for( unsigned i = 0; i < dthetas.size(); i++ ) {
      if( dthetas[i] > mean + sigma3 ) {
        printf( "%d : jump at %d, dtheta[%f]\n", ++ct, i+1, dthetas[i] );
        flips.push_back( dthetas[i] );
      }
    }

    double totalflip_dtheta = 0.0;
    for( unsigned i = 0; i < flips.size(); i++ ) {
      totalflip_dtheta += flips[i];
    }
    double meanflip_dtheta = totalflip_dtheta / (double)flips.size();
    printf( "flips[%d], meanflip_dtheta[%f]\n", flips.size(), meanflip_dtheta );
  }

  // Interpolates position and rotation on all vicon frames that were 
  // identified by evaluate() to correct the positional and orientation errors 
  // attributed to vicon when the 2d markers were occluded and the system 
  // mistakenly reoriented the shell which caused both jumps in position and 
  // orientation.
  void interpolate( void ) {
    std::vector<std::pair<unsigned,unsigned> > intervals;

    // Note: the start frames here are guidelines and may not fully reflect the 
    // values stored in the data files.  Consult the relevant file in 
    // data/interpreted/signals/trial_<I>.txt for the actual start frame

    if( _id == 1 ) {
      // start is video frame 54
      intervals.push_back( std::pair<unsigned,unsigned>( 352, 354 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 1733, 1734 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 11911, 11912 ) );
    } else if( _id == 2 ) {
      // start is video frame 36
      intervals.push_back( std::pair<unsigned,unsigned>( 1367, 1368 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 10133, 10134 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 12153, 12154 ) );
    } else if( _id == 3 ) {
      //intervals.push_back( std::pair<unsigned,unsigned>( 14, 15 ) );
      // start is video frame 100
      intervals.push_back( std::pair<unsigned,unsigned>( 477, 481 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 3046, 3068 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 3404, 3406 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 5981, 5982 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 7843, 7845 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 9600, 9601 ) );
    } else if( _id == 4 ) {
      // start is video frame 47
      intervals.push_back( std::pair<unsigned,unsigned>( 984, 986 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 2699, 2700 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 6791, 6792 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 6975, 6976 ) );
    } else if( _id == 5 ) {
      // start is video frame 25
      intervals.push_back( std::pair<unsigned,unsigned>( 20, 22 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 33, 35 ) );
      //4037 ?
      intervals.push_back( std::pair<unsigned,unsigned>( 11216, 11218 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 11227, 11228 ) );
    } else if( _id == 6 ) {
      // start is video frame 66
      intervals.push_back( std::pair<unsigned,unsigned>( 2381, 2383 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 2425, 2427 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 5573, 5576 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 9104, 9105 ) );
    } else if( _id == 7 ) {
      //intervals.push_back( std::pair<unsigned,unsigned>( 0, 7 ) );
      // start is video frame 45
      intervals.push_back( std::pair<unsigned,unsigned>( 16, 19 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 1103, 1105 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 1179, 1180 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 4423, 4425 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 4489, 4491 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 4855, 4856 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 4987, 4988 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 9757, 9758 ) );
    } else if( _id == 8 ) {
      // start is video frame 15
      intervals.push_back( std::pair<unsigned,unsigned>( 5362, 5363 ) );
      //intervals.push_back( std::pair<unsigned,unsigned>( 8124, 8126 ) );
      //intervals.push_back( std::pair<unsigned,unsigned>( 8128, 8130 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 8124, 8130 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 9561, 9562 ) );
    } else if( _id == 9 ) {
      // start is video frame 37
      intervals.push_back( std::pair<unsigned,unsigned>( 2638, 2642 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 3774, 3775 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 4280, 4282 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 5681, 5682 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 6435, 6438 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 9476, 9478 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 10869, 10871 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 10958, 10959 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 12167, 12169 ) );
    } else if( _id == 10 ) {
      // start is video frame 32
      intervals.push_back( std::pair<unsigned,unsigned>( 232, 233 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 2714, 2715 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 7168, 7169 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 8184, 8185 ) );
      intervals.push_back( std::pair<unsigned,unsigned>( 11940, 11942 ) );
    } else {
      return;
    }

    unsigned corrections = 0;
    for( std::vector< std::pair<unsigned, unsigned> >::iterator it = intervals.begin(); it != intervals.end(); it++ ) {
      unsigned idx_initial = it->first;
      unsigned idx_final = it->second + 1;
      unsigned steps = it->second - it->first;
      corrections += steps;

      wb_shell_state_ptr initial = shell_states[idx_initial-1];
      wb_shell_state_ptr final = shell_states[idx_final-1];
      double dt = 1.0 / VICON_SAMPLE_RATE;

      double dxdt, dydt, dzdt;
      dxdt = (final->val(0) - initial->val(0)) / (dt * (double)(steps+1));
      dydt = (final->val(1) - initial->val(1)) / (dt * (double)(steps+1));
      dzdt = (final->val(2) - initial->val(2)) / (dt * (double)(steps+1));

      Ravelin::Quatd q0, q1;
      q0 = Ravelin::Quatd( initial->val(3), initial->val(4), initial->val(5), initial->val(6) );
      q0.normalize();
      q1 = Ravelin::Quatd( final->val(3), final->val(4), final->val(5), final->val(6) );
      q1.normalize();

      printf( "steps[%d], initial[%d], final[%d]\n", steps, idx_initial, idx_final );

      for( unsigned i = 0; i < steps; i++ ) {
        unsigned idx = idx_initial + i+1;
        wb_shell_state_ptr current = shell_states[idx-1];
 
        // linear interpolation
        current->val( 0 ) = dxdt * (double)(i+1) * dt + initial->val(0);
        current->val( 1 ) = dydt * (double)(i+1) * dt + initial->val(1);
        current->val( 2 ) = dzdt * (double)(i+1) * dt + initial->val(2);

        // rotational interpolation (slerp)
        double alpha = (double)(i+1)/(double)(steps+1);
        Ravelin::Quatd q = Ravelin::Quatd::slerp( q0, q1, alpha );
        q.normalize();

        current->val( 3 ) = q.x;
        current->val( 4 ) = q.y;
        current->val( 5 ) = q.z;
        current->val( 6 ) = q.w;

        printf( "interpolating state[%d], alpha[%f]\n", idx, alpha );
      }
    }
    printf( "orientation corrections[%d]\n", corrections );
  }
};
//-----------------------------------------------------------------------------


/*
//-----------------------------------------------------------------------------
// Forward declarations of state classes
class wb_vicon_state_c;
class wb_fused_state_c;
class wb_full_state_c;

//-----------------------------------------------------------------------------
// Forward declarations of data classes
class wb_vicon_data_c;
class wb_fused_data_c;
class wb_full_data_c;

//-----------------------------------------------------------------------------
// Pointer aliases of state classes
typedef boost::shared_ptr<wb_vicon_state_c> wb_vicon_state_ptr;
typedef boost::shared_ptr<wb_fused_state_c> wb_fused_state_ptr;
typedef boost::shared_ptr<wb_full_state_c> wb_full_state_ptr;

//-----------------------------------------------------------------------------
// Pointer aliases of data classes
typedef boost::shared_ptr<wb_vicon_data_c> wb_vicon_data_ptr;
typedef boost::shared_ptr<wb_fused_data_c> wb_fused_data_ptr;
typedef boost::shared_ptr<wb_full_data_c> wb_full_data_ptr;

//-----------------------------------------------------------------------------
class wb_vicon_state_c : public state_c {
public:
  wb_vicon_state_c( void ) : state_c(7) {
    _x[6] = 1.0;  // normalize w coordinate
  }
  virtual ~wb_vicon_state_c( void ) {}

  void print( void ) {
    state_c::print( "wb_vicon_state" );
  }
};

//-----------------------------------------------------------------------------
class wb_fused_state_c : public state_c {
public:
  wb_fused_state_c( void ) : state_c(8) {
    _x[6] = 1.0;  // normalize w coordinate
  }
  virtual ~wb_fused_state_c( void ) {}

  void print( void ) {
    state_c::print( "wb_fused_state" );
  }
};

//-----------------------------------------------------------------------------
class wb_full_state_c : public state_c {
public:
  wb_full_state_c( void ) : state_c(26) {
    _x[6] = 1.0;  // normalize w coordinate
    _x[16] = 1.0;  // normalize w coordinate  // ? what index 16 or 19?
  }
  virtual ~wb_full_state_c( void ) {}

  void print( void ) {
    state_c::print( "wb_full_state" );
  }
};

//-----------------------------------------------------------------------------
class wb_vicon_data_c {
public:
  std::vector< std::vector<wb_vicon_state_ptr> > states;

  wb_vicon_data_c( void ) {
    states.resize( VICON_SESSIONS );
  }

  virtual ~wb_vicon_data_c( void ) {}

  bool load_mocap( void ) {
    std::string path = MOCAP_DATA_PATH;

    for( unsigned i = 0; i < VICON_SESSIONS; i++ ) {
      std::stringstream ssfile;
      unsigned fileid = i + 1;
      ssfile << path << "trial_" << std::setfill('0') << std::setw(2) << fileid << ".txt";
      printf( "reading file: %s\n", ssfile.str().c_str() );
      if( !read_vicon( ssfile.str(), states[i] ) )
        printf( "failed to read file: %s\n", ssfile.str().c_str() );
      printf( "states loaded: %u\n", states[i].size() );
    }
  }

  bool read_vicon( std::string filename, std::vector<wb_vicon_state_ptr>& states ) {
    std::string data;
    std::ifstream file( filename.c_str() );
    if( !file.is_open() ) return false;

    int min, sec, ms;
    double pos[3];
    double rot[4];

    int line = 0;
    while( std::getline( file, data ) ) {
      int idx = line++ % 4;
      if( idx == 0 ) {
        //timexxx: min:sec:ms [ex: time(min:sec:ms): 30:23:971]
        //prefix is fixed length so will only parse beginning at space
        size_t start = 18, end;
        std::string values = data.substr( start );
        end = values.find( ":" );
        min = atoi( values.substr( 0, end ).c_str() );
        start = end + 1;
        end = values.find( ":", start );
        sec = atoi( values.substr( start, end ).c_str() );
        start = end + 1;
        ms = atoi( values.substr( start ).c_str() );
      } else if( idx == 1 ) {
        //pos: x, y, z [ex: pos: -0.242842, 1.39527, 1.35857]
        //prefix is fixed length so will only parse beginning at first space
        size_t start = 5, end;
        std::string values = data.substr( start );
        end = values.find( "," );
        pos[0] = atof( values.substr( 0, end ).c_str() );
        start = end + 1;
        end = values.find( ",", start );
        pos[1] = atof( values.substr( start, end ).c_str() );
        start = end + 1;
        pos[2] = atof( values.substr( start ).c_str() );
      } else if( idx == 2 ) {
        //quat: x, y, z, w [ex. quat: 0.817646, -0.535734, 0.145617, -0.15245]
        //prefix is fixed length so will only parse beginning at first space
        size_t start = 6, end;
        std::string values = data.substr( start );
        end = values.find( "," );
        rot[0] = atof( values.substr( 0, end ).c_str() );
        start = end + 1;
        end = values.find( ",", start );
        rot[1] = atof( values.substr( start, end ).c_str() );
        start = end + 1;
        end = values.find( ",", start );
        rot[2] = atof( values.substr( start, end ).c_str() );
        start = end + 1;
        rot[3] = atof( values.substr( start ).c_str() );
      } else if( idx == 3 ) {
        // blank line
        // retain the record
 
        //std::cout << "(" << min << ":" << sec << ":" << ms << "):";
        //std::cout << "[" << pos[0] << "," << pos[1] << "," << pos[2] << ",";
        //std::cout << rot[0] << "," << rot[1] << "," << rot[2] << "," << rot[3] << "]" << std::endl;

        wb_vicon_state_ptr state = wb_vicon_state_ptr( new wb_vicon_state_c() );

        state->t( min, sec, ms );
        for( unsigned i = 0; i < 3; i++ )
          state->val(i) = pos[i];
        for( unsigned i = 0; i < 4; i++ )
          state->val(i+3) = rot[i];
        states.push_back( state );

        //state->print();
        //printf( "\n" );
      }

      //std::cout << data << std::endl;

    }
    file.close();

    return true;
  }
};

//-----------------------------------------------------------------------------
// Forward declarations of state classes
class wb_led_state_c;
//-----------------------------------------------------------------------------
// Forward declarations of data classes
class wb_led_data_c;
//-----------------------------------------------------------------------------
// Pointer aliases of state classes
typedef boost::shared_ptr<wb_led_state_c> wb_led_state_ptr;
//-----------------------------------------------------------------------------
// Pointer aliases of data classes
typedef boost::shared_ptr<wb_led_data_c> wb_led_data_ptr;
//-----------------------------------------------------------------------------
class wb_led_data_c {
public:
  std::vector< std::vector<wb_led_state_ptr> > states;

  wb_led_data_c( void ) {
    states.resize( VICON_SESSIONS );
  }

  virtual ~wb_led_data_c( void ) {}

  bool load_mocap( void ) {
    std::string path = MOCAP_DATA_PATH;

    for( unsigned i = 0; i < VICON_SESSIONS; i++ ) {
      std::stringstream ssfile;
      unsigned fileid = i + 1;
      ssfile << path << "trial_" << std::setfill('0') << std::setw(2) << fileid << ".txt";
      printf( "reading file: %s\n", ssfile.str().c_str() );
      if( !read_vicon( ssfile.str(), states[i] ) )
        printf( "failed to read file: %s\n", ssfile.str().c_str() );
      printf( "states loaded: %u\n", states[i].size() );
    }
  }

  bool read_vicon( std::string filename, std::vector<wb_vicon_state_ptr>& states ) {
    std::string data;
    std::ifstream file( filename.c_str() );
    if( !file.is_open() ) return false;

    int min, sec, ms;
    double pos[3];
    double rot[4];

    int line = 0;
    while( std::getline( file, data ) ) {
      int idx = line++ % 4;
      if( idx == 0 ) {
        //timexxx: min:sec:ms [ex: time(min:sec:ms): 30:23:971]
        //prefix is fixed length so will only parse beginning at space
        size_t start = 18, end;
        std::string values = data.substr( start );
        end = values.find( ":" );
        min = atoi( values.substr( 0, end ).c_str() );
        start = end + 1;
        end = values.find( ":", start );
        sec = atoi( values.substr( start, end ).c_str() );
        start = end + 1;
        ms = atoi( values.substr( start ).c_str() );
      } else if( idx == 1 ) {
        //pos: x, y, z [ex: pos: -0.242842, 1.39527, 1.35857]
        //prefix is fixed length so will only parse beginning at first space
        size_t start = 5, end;
        std::string values = data.substr( start );
        end = values.find( "," );
        pos[0] = atof( values.substr( 0, end ).c_str() );
        start = end + 1;
        end = values.find( ",", start );
        pos[1] = atof( values.substr( start, end ).c_str() );
        start = end + 1;
        pos[2] = atof( values.substr( start ).c_str() );
      } else if( idx == 2 ) {
        //quat: x, y, z, w [ex. quat: 0.817646, -0.535734, 0.145617, -0.15245]
        //prefix is fixed length so will only parse beginning at first space
        size_t start = 6, end;
        std::string values = data.substr( start );
        end = values.find( "," );
        rot[0] = atof( values.substr( 0, end ).c_str() );
        start = end + 1;
        end = values.find( ",", start );
        rot[1] = atof( values.substr( start, end ).c_str() );
        start = end + 1;
        end = values.find( ",", start );
        rot[2] = atof( values.substr( start, end ).c_str() );
        start = end + 1;
        rot[3] = atof( values.substr( start ).c_str() );
      } else if( idx == 3 ) {
        // blank line
        // retain the record
 
        //std::cout << "(" << min << ":" << sec << ":" << ms << "):";
        //std::cout << "[" << pos[0] << "," << pos[1] << "," << pos[2] << ",";
        //std::cout << rot[0] << "," << rot[1] << "," << rot[2] << "," << rot[3] << "]" << std::endl;

        wb_vicon_state_ptr state = wb_vicon_state_ptr( new wb_vicon_state_c() );

        state->t( min, sec, ms );
        for( unsigned i = 0; i < 3; i++ )
          state->val(i) = pos[i];
        for( unsigned i = 0; i < 4; i++ )
          state->val(i+3) = rot[i];
        states.push_back( state );

        //state->print();
        //printf( "\n" );
      }

      //std::cout << data << std::endl;

    }
    file.close();

    return true;
  }

};
*/
/*
//-----------------------------------------------------------------------------
class wb_fused_data_c {
public:
  std::vector< std::vector<wb_fused_state_ptr> > states;

  wb_fused_data_c( void ) {
    states.resize( VICON_SESSIONS );
  }

  virtual ~wb_fused_data_c( void ) {}

  bool load( void ) {
    std::string path = "/home/james/weazel-experiment/data/";

    for( unsigned i = 0; i < VICON_SESSIONS; i++ ) {
      std::stringstream ssfile;
      unsigned fileid = i + 1;
      ssfile << path << "trial_" << std::setfill('0') << std::setw(2) << fileid << ".log";
      printf( "reading file: %s\n", ssfile.str().c_str() );
      if( !read_log( ssfile.str(), states[i] ) )
        printf( "failed to read file: %s\n", ssfile.str().c_str() );
      printf( "states loaded: %u\n", states[i].size() );
    }
  }

  bool read_log( std::string filename, std::vector<wb_fused_state_ptr>& states ) {
    std::string data;
    std::ifstream file( filename.c_str() );
    if( !file.is_open() ) return false;

    double t;
    double pos[3];
    double rot[4];
    double angle;

    int line = 0;
    while( std::getline( file, data ) ) {
      unsigned i = 0;
      boost::char_separator<char> delim(" ");
      boost::tokenizer< boost::char_separator<char> > tokenizer( data, delim );
      //for( boost::tokenizer< char_separator<char> >::iterator it = tokenizer.begin(); it != tokenizer.end(); ++it ) {
      BOOST_FOREACH( const std::string& token, tokenizer ) {
        //std::string token = *it;
        //printf( "%s ", token.c_str() );
        switch( i++ ) {
        default:
        case 0:
          t = atof( token.c_str() );
          break;
        case 1:
          pos[0] = atof( token.c_str() );
          break;
        case 2:
          pos[1] = atof( token.c_str() );
          break;
        case 3:
          pos[2] = atof( token.c_str() );
          break;
        case 4:
          rot[0] = atof( token.c_str() );
          break;
        case 5:
          rot[1] = atof( token.c_str() );
          break;
        case 6:
          rot[2] = atof( token.c_str() );
          break;
        case 7:
          rot[3] = atof( token.c_str() );
          break;
        case 8:
          angle = atof( token.c_str() );
          break;
        }
      }
      //printf( "\n" );

      wb_fused_state_ptr state = wb_fused_state_ptr( new wb_fused_state_c() );

      state->t( t );
      for( unsigned i = 0; i < 3; i++ )
        state->val(i) = pos[i];
      for( unsigned i = 0; i < 4; i++ )
        state->val(i+3) = rot[i];
      state->val(7) = angle;

      states.push_back( state );

      line++;
    }
    file.close();

    return true;
  }
};
*/
/*
//-----------------------------------------------------------------------------
class wb_full_data_c {
public:
  std::vector< std::vector<wb_full_state_ptr> > states;

  wb_full_data_c( void ) {
    states.resize( VICON_SESSIONS );
  }

  virtual ~wb_full_data_c( void ) {}

//  bool load_mocap( void ) {
//    //std::string path = "/home/j/weasel-experiment-01-14-15/trials/";
//    std::string path = "/home/james/weazel-experiment/trials/";
//
//    for( unsigned i = 0; i <= 9; i++ ) {
//    //for( unsigned i = 1; i <= 9; i++ ) {
//      std::stringstream ssfile;
//      unsigned fileid = i + 1;
//      ssfile << path << "trial" << fileid << ".txt";
//      printf( "reading file: %s\n", ssfile.str().c_str() );
//      if( !read_vicon( ssfile.str(), states[i] ) )
//        printf( "failed to read file: %s\n", ssfile.str().c_str() );
//      printf( "states loaded: %u\n", states[i].size() );
//    }
//  }
//
//  bool read_vicon( std::string filename, std::vector<wb_fused_state_ptr>& states ) {
//    std::string data;
//    //std::string datapath = "/home/j/weasel-experiment-01-14-15/trials/";
//    //std::string datafile = "trial1.txt";
//    //std::string filename = datapath + datafile;
//    std::ifstream file( filename.c_str() );
//    if( !file.is_open() ) return false;
//
//    int min, sec, ms;
//    double pos[3];
//    double rot[4];
//
//    int line = 0;
//    while( std::getline( file, data ) ) {
//      int idx = line++ % 4;
//      if( idx == 0 ) {
//        //timexxx: min:sec:ms [ex: time(min:sec:ms): 30:23:971]
//        //prefix is fixed length so will only parse beginning at space
//        size_t start = 18, end;
//        std::string values = data.substr( start );
//        end = values.find( ":" );
//        min = atoi( values.substr( 0, end ).c_str() );
//        start = end + 1;
//        end = values.find( ":", start );
//        sec = atoi( values.substr( start, end ).c_str() );
//        start = end + 1;
//        ms = atoi( values.substr( start ).c_str() );
//      } else if( idx == 1 ) {
//        //pos: x, y, z [ex: pos: -0.242842, 1.39527, 1.35857]
//        //prefix is fixed length so will only parse beginning at first space
//        size_t start = 5, end;
//        std::string values = data.substr( start );
//        end = values.find( "," );
//        pos[0] = atof( values.substr( 0, end ).c_str() );
//        start = end + 1;
//        end = values.find( ",", start );
//        pos[1] = atof( values.substr( start, end ).c_str() );
//        start = end + 1;
//        pos[2] = atof( values.substr( start ).c_str() );
//      } else if( idx == 2 ) {
//        //quat: x, y, z, w [ex. quat: 0.817646, -0.535734, 0.145617, -0.15245]
//        //prefix is fixed length so will only parse beginning at first space
//        size_t start = 6, end;
//        std::string values = data.substr( start );
//        end = values.find( "," );
//        rot[0] = atof( values.substr( 0, end ).c_str() );
//        start = end + 1;
//        end = values.find( ",", start );
//        rot[1] = atof( values.substr( start, end ).c_str() );
//        start = end + 1;
//        end = values.find( ",", start );
//        rot[2] = atof( values.substr( start, end ).c_str() );
//        start = end + 1;
//        rot[3] = atof( values.substr( start ).c_str() );
//      } else if( idx == 3 ) {
//        // blank line
//        // retain the record
// 
//        //std::cout << "(" << min << ":" << sec << ":" << ms << "):";
//        //std::cout << "[" << pos[0] << "," << pos[1] << "," << pos[2] << ",";
//        //std::cout << rot[0] << "," << rot[1] << "," << rot[2] << "," << rot[3] << "]" << std::endl;
//
//        wb_fused_state_ptr state = wb_fused_state_ptr( new wb_fused_state_c() );
//
//        state->t( min, sec, ms );
//        for( unsigned i = 0; i < 3; i++ )
//          state->val(i) = pos[i];
//        for( unsigned i = 0; i < 4; i++ )
//          state->val(i+3) = rot[i];
//        states.push_back( state );
//
//        //state->print();
//        //printf( "\n" );
//      }
//
//      //std::cout << data << std::endl;
//
//    }
//    file.close();
//
//    return true;
//  }

};
*/

#endif // _WEAZELBALL_H_
