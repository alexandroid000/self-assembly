#ifndef _WEAZELBALL_MODELS_H_
#define _WEAZELBALL_MODELS_H_

//-----------------------------------------------------------------------------
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include "vicon.h"
#include "weazelball.h"

//-----------------------------------------------------------------------------
using namespace gazebo;

//-----------------------------------------------------------------------------

class model_c;
typedef boost::shared_ptr< model_c > model_ptr;

class weazelball_c;
typedef boost::shared_ptr< weazelball_c > weazelball_ptr;

class world_c;
typedef boost::shared_ptr< world_c > world_ptr;

//-----------------------------------------------------------------------------
class model_c
{
protected:
  physics::WorldPtr _world;
  physics::ModelPtr _model;

  physics::ModelPtr model( std::string name ) {
    return _world->GetModel( name );
  }

  physics::LinkPtr link( std::string name ) {
    return _model->GetLink( name );
  }

  physics::JointPtr joint( std::string name ) {
    return _model->GetJoint( name );
  }

  physics::ModelPtr model( std::string name, std::string& errors ) {
    physics::ModelPtr m = model( name );
    if( !m ) errors += "Unable to find model: " + name + "\n";
    return m;
  }

  physics::LinkPtr link( std::string name, std::string& errors ) {
    physics::LinkPtr l = link( name );
    if( !l ) errors += "Unable to find link: " + name + "\n";
    return l;
  }

  physics::JointPtr joint( std::string name, std::string& errors ) {
    physics::JointPtr j = joint( name );
    if( !j ) errors += "Unable to find joint: " + name + "\n";
    return j;
  }

public:
  model_c( physics::WorldPtr world ) : _world( world ) { }
  model_c( physics::ModelPtr model ) : _model( model ) { _world = _model->GetWorld(); }
  virtual ~model_c( void ) { }

  physics::Link_V links( void ) { return _model->GetLinks(); }
  physics::Joint_V joints( void ) { return _model->GetJoints(); }
  physics::ModelPtr model( void ) { return _model; }
  std::string name( void ) { return _model->GetName(); }

};

//-----------------------------------------------------------------------------
class weazelball_c : public model_c
{
private:
  physics::LinkPtr _shell; 
  physics::LinkPtr _motor;
  
  physics::JointPtr _actuator;

  math::Vector3 center_offset;
  math::Quaternion rotation_offset;

  std::vector<math::Vector3> marker_positions;
public:

  physics::LinkPtr shell( void ) { return _shell; }
  physics::LinkPtr motor( void ) { return _motor; }
  
  physics::JointPtr actuator( void ) { return _actuator; }

public:
  weazelball_c( physics::WorldPtr world ) : model_c( world ) { 
      std::cout << "vsk res: " << read_vsk() << std::endl;
  }
  weazelball_c( physics::ModelPtr model ) : model_c( model ) { 
  }
  virtual ~weazelball_c( void ) { }

  math::Vector3 get_center_offset( void ) {
    // unadjusted offset center 
    //return math::Vector3(0, 0, 0) / 1000;

    // offset center computed by perturbation
    return math::Vector3(-1.066979004925168, 3.498590099946762, 2.312719577988714) / 1000;

    // offset center computed by minimization
    //return math::Vector3(-1.266714124699, 3.365409558160, 1.907292996249) / 1000; 
  }

  math::Quaternion get_rotation_offset( void ) {
    //return gazebo::math::Quaternion( -PI/2.0, 0.0, 0 ) * gazebo::math::Quaternion( 0, 0, PI );
    return gazebo::math::Quaternion( -PI/2.0, 0.0, 0 );
  }

  bool validate( std::string& errors ) {
    std::string id;

    if( _model ) {
      _world = _model->GetWorld();
    } else if( _world ) {
      _model = model_c::model( "weazelball", errors );
      if( !_model )
	  {
		  errors += "Something fucked\n";
          return false;
	  }
    } else {
      errors += "No world or model provided to weazelball_c instance\n";
      return false;
    }

    _shell = link( "shell", errors );
    _motor = link( "motor", errors );
    _actuator = joint( "motor_actuator", errors );

    if( !( _world && _model && _shell && _motor && _actuator ) ) 
      return false;
    return true;
  }

  void pose( wb_vicon_state_ptr state ) {
    // extract the position vector from the state
    gazebo::math::Vector3 pos = gazebo::math::Vector3( state->val(0), state->val(1), state->val(2) );
    //pos.z = 0.041;

    //pos += center_offset;

    // extract the rotation quaternion from the state
    gazebo::math::Quaternion rot = gazebo::math::Quaternion( state->val(6), state->val(3), state->val(4), state->val(5) );

    center_offset = rot.RotateVector( get_center_offset() );

    // the world frame.  Rotate the model frame back to a frame aligned with 
    // the world.
    //rot = rot * gazebo::math::Quaternion( -PI/2.0, 0.0, 0.0 );
    rot = rot * get_rotation_offset();

    //gazebo::math::Quaternion q( 0, 0, PI);
    //center_offset = rot.RotateVector( q.RotateVector(math::Vector3(-1.0671, 3.4986, 2.3128) / 1000) );
    //gazebo::math::Quaternion q( 0, 0, PI);
    //center_offset = rot.RotateVector( math::Vector3(-1.0671, 3.4986, 2.3128) / 1000);
    //math::Vector3 co = rot.RotateVector( center_offset );

    pos += center_offset;

    // update the pose of the shell only.  Have no reference for motor in vicon
    _model->SetLinkWorldPose( gazebo::math::Pose( pos, rot ), _shell );

    state->val(0) = pos.x;
    state->val(1) = pos.y;
    state->val(2) = pos.z;
    state->val(3) = rot.x;
    state->val(4) = rot.y;
    state->val(5) = rot.z;
    state->val(6) = rot.w;
  }

  void pose( wb_shell_state_ptr state ) {
    // extract the position vector from the state
    gazebo::math::Vector3 pos = gazebo::math::Vector3( state->val(0), state->val(1), state->val(2) );
    //pos.z = 0.041;

    //pos += center_offset;

    // extract the rotation quaternion from the state
    gazebo::math::Quaternion rot = gazebo::math::Quaternion( state->val(6), state->val(3), state->val(4), state->val(5) );

    //math::Vector3 co = rot.RotateVector( math::Vector3(-1.0671, 3.4986, 2.3128) / 1000 );
    //math::Vector3 co = rot.RotateVector( center_offset );

    //center_offset = rot.RotateVector( math::Vector3(-1.0671, 3.4986, 2.3128) / 1000 );
    // the vicon data is recorded with a model frame rotated by 90 degrees to 

    center_offset = rot.RotateVector( get_center_offset() );

    // the world frame.  Rotate the model frame back to a frame aligned with 
    // the world.
    //rot = rot * gazebo::math::Quaternion( -PI/2.0, 0.0, 0.0 );
    rot = rot * get_rotation_offset();

    //gazebo::math::Quaternion q( 0, 0, PI);
    //center_offset = rot.RotateVector( q.RotateVector(math::Vector3(-1.0671, 3.4986, 2.3128) / 1000) );
    //center_offset = rot.RotateVector( math::Vector3(-1.0671, 3.4986, 2.3128) / 1000 );
    //math::Vector3 co = rot.RotateVector( center_offset );

    pos += center_offset;
//    pos -= co;

    // update the pose of the shell only.  Have no reference for motor in vicon
    _model->SetLinkWorldPose( gazebo::math::Pose( pos, rot ), _shell );

    state->val(0) = pos.x;
    state->val(1) = pos.y;
    state->val(2) = pos.z;
    state->val(3) = rot.x;
    state->val(4) = rot.y;
    state->val(5) = rot.z;
    state->val(6) = rot.w;
  }

  bool read_vsk( void ) {
    TiXmlDocument doc;
    std::string path = MOCAP_DATA_PATH;
    std::stringstream ssfile;
    ssfile << path << "wb.vsk";
    printf( "reading file: %s\n", ssfile.str().c_str() );

    if( !doc.LoadFile( ssfile.str() ) ) 
	{
	  std::cout << "ERROR: LoadFile Failed!" << std::endl;
	  return false;
	}

    TiXmlElement *modelxml = doc.FirstChildElement( "KinematicModel" );
    if (!modelxml) {
      std::cout << "ERROR: No <KinematicModel> element.  Cannot parse vsk file[" << path << "].  Markers will not be visualized.\n";
      return false;
    }

    TiXmlElement *markersetxml = modelxml->FirstChildElement( "MarkerSet" );
    if (!markersetxml) {
      std::cout << "ERROR: No <MarkerSet> element.  Cannot parse vsk file[" << path << "].  Markers will not be visualized.\n";
      return false;
    }

    TiXmlElement *markersxml = markersetxml->FirstChildElement( "Markers" );
    if (!markersxml) {
      std::cout << "ERROR: No <Markers> element.  Cannot parse vsk file[" << path << "].  Markers will not be visualized.\n";
      return false;
    }

    TiXmlElement *markerxml, *prevmarkerxml;
    markerxml = markersxml->FirstChildElement( "Marker" );
    if (!markerxml) {
      std::cout << "ERROR: No <Marker> element.  Cannot parse vsk file[" << path << "].  Markers will not be visualized.\n";
      return false;
    }

    while( markerxml ) {
      std::string positionxml = markerxml->Attribute( "POSITION" );
      if ( !positionxml.size() ) {
        std::cout << "ERROR: No \"POSITION\" attribute in <Marker>.  Ignoring element.  Malformed marker will not be visualized.\n";
      } else {
        math::Vector3 pos;
        std::stringstream ss( positionxml );
        std::istream_iterator<std::string> begin( ss );
        std::istream_iterator<std::string> end;
        std::vector<std::string> vstrings( begin, end );
        if( vstrings.size() == 3 ) {
          pos.x = atof( vstrings[0].c_str() );
          pos.y = atof( vstrings[1].c_str() );
          pos.z = atof( vstrings[2].c_str() );
          pos /= 1000;
          std::cout << "position: " << pos << std::endl;
          marker_positions.push_back(pos);
        }
      }

      prevmarkerxml = markerxml;
      markerxml = prevmarkerxml->NextSiblingElement( "Marker" );
    }

    return true;
  }
};

//-----------------------------------------------------------------------------
class led_c;
typedef boost::shared_ptr< led_c > led_ptr;

class led_c : protected model_c 
{
private:
  physics::ModelPtr _led_off;
  physics::ModelPtr _led_on;

  bool _on;

public:
  led_c( physics::WorldPtr world ) : model_c( world ) { _on = false; }

  bool validate( std::string& errors ) {
    std::string id;

    if( _world ) {
      _led_off = model_c::model( "weazelsignaloff", errors );
      if( !_led_off ) return false;
      _led_on = model_c::model( "weazelsignalon", errors );
      if( !_led_on ) return false;
    } else {
      errors += "No world or model provided to led_c instance\n";
      return false;
    }
/*
    _shell = link( "shell", errors );
    _motor = link( "motor", errors );
    _actuator = joint( "motor_actuator", errors );
*/
    if( !( _world && _led_off && _led_on ) ) 
      return false;
    return true;
  }

  void switch_on( void ) {
    gazebo::math::Vector3 on_pos( 0, 0, 0.25 );
    gazebo::math::Vector3 off_pos( 0, 0, -0.25 );
    gazebo::math::Quaternion rot( 1, 0, 0, 0 );

    _led_on->SetLinkWorldPose( gazebo::math::Pose( on_pos, rot ), "geom" );
    _led_off->SetLinkWorldPose( gazebo::math::Pose( off_pos, rot ), "geom" ); 
  }

  void switch_off( void ) {
    gazebo::math::Vector3 on_pos( 0, 0, -0.25 );
    gazebo::math::Vector3 off_pos( 0, 0, 0.25 );
    gazebo::math::Quaternion rot( 1, 0, 0, 0 );

    _led_on->SetLinkWorldPose( gazebo::math::Pose( on_pos, rot ), "geom" );
    _led_off->SetLinkWorldPose( gazebo::math::Pose( off_pos, rot ), "geom" );

  }
};

//-----------------------------------------------------------------------------
class world_c 
{
private:
  physics::WorldPtr _world; 

  std::vector<weazelball_ptr> _weazelballs;

  led_ptr _led;

  unsigned trial_index;

  bool _has_led;

public:
  world_c( physics::WorldPtr world ) : _world( world ) { _has_led = false; }
  virtual ~world_c( void ) { }

  physics::WorldPtr gzworld( void ) { return _world; }
  std::vector<weazelball_ptr> weazelballs( void ) { return _weazelballs; }
  led_ptr led( void ) { return _led; }

  bool validate( std::string& errors ) {
    std::string validation_errors;
    bool weazelball_validated, led_validated;
    errors = "";

    // create the weazelball model encapsulation structure and validate it 
	for(int i = 0; i < 4; i++)
	{
		weazelball_ptr weazelball_temp = weazelball_ptr( new weazelball_c( _world ) );
	//	weazelball_validated = weazelball_temp->validate( validation_errors );
		_weazelballs.push_back(weazelball_temp);
	/*	if( !( weazelball_validated ) ) 
		{
			std::cout << "Weaselball " << i << " failed!" << std::endl;
			return false;
		}*/
	}

    _led = led_ptr( new led_c( _world ) );
    led_validated = _led->validate( validation_errors );
    if( !( led_validated ) ) _has_led = true;
  
    return true;
  }

  void reset( void ) {
    _world->Reset();
  }

  double sim_time( void ) {
    return _world->GetSimTime().Double();
  }
 
  void sim_time( double t ) {
    _world->SetSimTime( common::Time( t ) );
  }

  double step_size( void ) {
    return _world->GetPhysicsEngine()->GetMaxStepSize();
  }

  double real_time( void ) {
    return _world->GetRealTime().Double();
  }
};

//-----------------------------------------------------------------------------
#endif // _WEAZELBALL_MODELS_H_
