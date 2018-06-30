/*
  State consists of shell(pos[3], rot[4], vel[3], drot[3], theta, dtheta)
  pos, rot, vel, drot are shell parameters
  theta and dtheta are joint parameters
*/

#ifndef _WB_GAZEBO_LOG_H_
#define _WB_GAZEBO_LOG_H_

//-----------------------------------------------------------------------------

#include "log.h"
#include "common.h"
#include "state.h"
#include "models.h"
#include "weazelball.h"

#include <vector>
#include <string>
#include <sstream>

#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>

//-----------------------------------------------------------------------------
// Forward declarations
class wb_state_c;
class wb_gazebo_session_c;
class wb_gazebo_data_c;
//-----------------------------------------------------------------------------
// Pointer aliases
typedef boost::shared_ptr<wb_state_c> wb_state_ptr;
typedef boost::shared_ptr<wb_gazebo_session_c> wb_gazebo_session_ptr;
typedef boost::shared_ptr<wb_gazebo_data_c> wb_gazebo_data_ptr;
//-----------------------------------------------------------------------------
class wb_state_c : public state_c {
public:
  wb_state_c( void ) : state_c(15) {
    _x[6] = 1.0;  // normalize w coordinate
  }
  virtual ~wb_state_c( void ) {}

  void print( void ) {
    state_c::print( "wb_state" );
  }
};
//-----------------------------------------------------------------------------
class wb_gazebo_session_c {
public:
  enum mode_e {
    READ,
    WRITE
  };
private:
  //unsigned _id;
  enum mode_e _mode;
  log_ptr _log;
public:
  std::vector<wb_state_ptr> states;

public:
  wb_gazebo_session_c( enum mode_e mode ) {
    //_id = 0;
    _mode = mode;
  }

  virtual ~wb_gazebo_session_c( void ) {
    close();
  }

  //unsigned id( void ) { return _id; } 

  int open( std::string name ) {
    assert( _mode == WRITE );

    std::stringstream ssname;
    ssname << name;
    _log = log_ptr( new log_c( ssname.str() ) );
    if( !_log || !_log->open() ) {
      return 1;
    }
    return 0;
  }

  void close( void ) {
    if( _log ) {
      _log->close();
      _log = log_ptr();
    }
  }

  int write( double virtual_t, weazelball_ptr wb ) {
    assert( _mode == WRITE );
    assert( _log );

    std::stringstream data;
    //gazebo::math::Vector3 pos = wb->shell()->GetWorldPose().pos;
    //gazebo::math::Quaternion rot = wb->shell()->GetWorldPose().rot;
    //double theta = wb->actuator()->GetAngle(0).Radian();

    //data << virtual_t << " ";
    //data << pos.x << " " << pos.y << " "<< pos.z << " ";
    //data << rot.x << " " << rot.y << " "<< rot.z << " " << rot.w << " ";
    //data << theta << std::endl;
 
    gazebo::physics::ModelPtr base = wb->model();
    gazebo::math::Pose pose = wb->shell()->GetWorldPose();

    gazebo::math::Vector3 pos = pose.pos;
    gazebo::math::Quaternion rot = pose.rot;
    gazebo::math::Vector3 vel = base->GetWorldLinearVel();
    gazebo::math::Vector3 drot = base->GetWorldAngularVel();
    double theta = wb->actuator()->GetAngle(0).Radian();
    //double dtheta = wb->actuator()->GetVelocity(0).Radian();
    double dtheta = wb->actuator()->GetVelocity(0);

    data << virtual_t << " ";
    data << pos.x << " " << pos.y << " "<< pos.z << " ";
    data << rot.x << " " << rot.y << " "<< rot.z << " " << rot.w << " ";
    data << vel.x << " " << vel.y << " "<< vel.z << " ";
    data << drot.x << " " << drot.y << " "<< drot.z << " ";
    data << theta << " " << dtheta << std::endl;
 
    if( !_log->write( data.str() ) ) {
      return 1;
    }
    return 0;
  }

  int cache( std::string filename ) {
    assert( _mode == READ );

    std::string data;
    char buf[512]; 
    std::ifstream file( filename.c_str() );
    if( !file.is_open() ) return 1;

    while( std::getline( file, data ) ) {
      strcpy(buf, data.c_str());
      wb_state_ptr state = wb_state_ptr( new wb_state_c() );
      char* gobble;
      gobble = strtok( buf, " " );
      state->t( atof(gobble) );
      int i = 0;
      while( gobble != NULL ) {
        //printf( "gobble:%s\n", gobble );
        gobble = strtok( NULL, " " );
        if( gobble != NULL )
          state->val( i++ ) = atof(gobble);  
      }
      states.push_back( state );
    }
    file.close();

    return 0;
  }

  int assign_state( world_ptr world, weazelball_ptr wb, int state_idx ) {
    if( state_idx > states.size() ) return 1;

    wb_state_ptr state = states.at(state_idx);

    // extract the position vector of the shell from the state
    gazebo::math::Vector3 pos = gazebo::math::Vector3( state->val(0), state->val(1), state->val(2) );

    // extract the rotation quaternion of the shell from the state
    gazebo::math::Quaternion rot = gazebo::math::Quaternion( state->val(6), state->val(3), state->val(4), state->val(5) );

    // extract the linear velocity of the shell from the state
    gazebo::math::Vector3 vel = gazebo::math::Vector3( state->val(7), state->val(8), state->val(9) );

    // extract the angular velocity of the shell from the state
    gazebo::math::Vector3 drot = gazebo::math::Vector3( state->val(10), state->val(11), state->val(12) );

    // extract the joint angle and the joint velocity from the state
    double theta = state->val(13);
    double dtheta = state->val(14);

    // map state to the model
    wb->model()->SetLinkWorldPose( gazebo::math::Pose( pos, rot ), wb->shell() );
    wb->model()->SetLinearVel( vel );
    wb->model()->SetAngularVel( drot );
    wb->actuator()->SetPosition( 0, theta );
    wb->actuator()->SetVelocity( 0, dtheta );

    //world->sim_time( state->t() );

/*
    std::cout << state->t() << " ";
    std::cout << pos.x << " " << pos.y << " " << pos.z << " ";
    std::cout << rot.x << " " << rot.y << " " << rot.z << " " << rot.w << " ";
    std::cout << theta << std::endl;       
*/
    return 0;
  }

  double get_t( int state_idx ) {
    return states.at(state_idx)->t();
  }
/*
  double get_dt( void ) {
    return states.at(1)->t() - states.at(0)->t();
  }
*/
  int size(void) {
    return states.size();
  }

  int print( int state_idx ) {
    states.at(state_idx)->print();
  }
};
#endif // _WB_GAZEBO_LOG_H_
