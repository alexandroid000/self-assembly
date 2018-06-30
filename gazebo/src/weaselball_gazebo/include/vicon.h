#ifndef _VICON_H_
#define _VICON_H_

//-----------------------------------------------------------------------------

#include "common.h"
#include "state.h"
#include <vector>
#include <string>
#include <sstream>

#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>

//-----------------------------------------------------------------------------
// Forward declarations
class wb_vicon_state_c;
class wb_vicon_session_c;
class wb_vicon_data_c;
//-----------------------------------------------------------------------------
// Pointer aliases
typedef boost::shared_ptr<wb_vicon_state_c> wb_vicon_state_ptr;
typedef boost::shared_ptr<wb_vicon_session_c> wb_vicon_session_ptr;
typedef boost::shared_ptr<wb_vicon_data_c> wb_vicon_data_ptr;
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
class wb_vicon_session_c {
private:
  unsigned _id;
public:
  std::vector<wb_vicon_state_ptr> states;
public:
  wb_vicon_session_c( void ) { _id = 0; }

  virtual ~wb_vicon_session_c( void ) {}

  unsigned id( void ) { return _id; } 

  static wb_vicon_session_ptr load( unsigned session_id ) {
    wb_vicon_session_ptr session;

    std::string path = MOCAP_DATA_PATH;

    std::stringstream ssfile;
    unsigned fileid = session_id;
    ssfile << path << "trial_" << std::setfill('0') << std::setw(2) << fileid << ".txt";
    printf( "reading file: %s\n", ssfile.str().c_str() );
    session = read_vicon_file( ssfile.str() );
    if( !session ) {
      printf( "failed to read file: %s\n", ssfile.str().c_str() );
    } else {
      session->_id = session_id;
      printf( "states loaded: %u\n", session->states.size() );
    }
    return session;
  }

  static wb_vicon_session_ptr read_vicon_file( std::string filename ) {
    wb_vicon_session_ptr session;

    std::string data;
    std::ifstream file( filename.c_str() );
    if( !file.is_open() ) return session;

    session = wb_vicon_session_ptr( new wb_vicon_session_c() );

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
#ifdef FILTER
        pos[2] = 0.041;  // filter correction : the ball never leaves the 
                         // surface of the table
#else
        pos[2] = atof( values.substr( start ).c_str() );
#endif
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
        session->states.push_back( state );

        //state->print();
        //printf( "\n" );
      }

      //std::cout << data << std::endl;

    }
    file.close();

    return session;
  }
};
//-----------------------------------------------------------------------------
class wb_vicon_data_c {
public:
  //std::vector< std::vector<wb_vicon_state_ptr> > states;
  std::vector< wb_vicon_session_ptr > sessions;

  wb_vicon_data_c( void ) { }

  virtual ~wb_vicon_data_c( void ) {}

  bool load_mocap( void ) {
    for( unsigned i = 0; i < EXPERIMENTAL_SESSIONS; i++ ) {
      wb_vicon_session_ptr session = wb_vicon_session_c::load( i+1 );
      if( session ) { 
        sessions.push_back( session );
      }
    }
  }
/*
  bool load_mocap( void ) {
    std::string path = MOCAP_DATA_PATH;

    for( unsigned i = 0; i < EXPERIMENTAL_SESSIONS; i++ ) {
      std::stringstream ssfile;
      unsigned fileid = i + 1;
      ssfile << path << "trial_" << std::setfill('0') << std::setw(2) << fileid << ".txt";
      printf( "reading file: %s\n", ssfile.str().c_str() );
      if( !read_vicon( ssfile.str(), states[i] ) )
        printf( "failed to read file: %s\n", ssfile.str().c_str() );
      printf( "states loaded: %u\n", states[i].size() );
    }
  }
*/
/*
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
        //pos[2] = atof( values.substr( start ).c_str() );
        pos[2] = 0.041;  // filter correction : the ball never leaves the 
                         // surface of the table
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
*/
};

#endif // _VICON_H_
