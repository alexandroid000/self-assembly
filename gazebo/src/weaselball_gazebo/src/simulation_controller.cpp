/*
This controller was cloned from the vicon visualization controller after 
refactoring.  It still contains logging code which was culled from the vicon
controller.  This controller can play the basis of another controller that
processes the data rather than modifying the vicon controller further.  This is
to ensure that a controller exists for each step in post-processing
*/
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

//#include "log.h"
#include "../include/models.h"
#include "../include/weazelball.h"
#include "../include/gazebo_log.h"

#include <stdlib.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <random>


//#include "movie.h"

#define PI 3.14159265359

//-----------------------------------------------------------------------------
/**
The gazebo plugin visualization controller
**/
namespace gazebo 
{
  class controller_c : public ModelPlugin
  {
  private:
    event::ConnectionPtr _updateConnection;  //< Gazebo update callback

    world_ptr _world;                        //< Gazebo world pointer
    weazelball_ptr _weazelball;              //< Weazelball data structure

    wb_vicon_data_ptr _wb_vicon_data;        //< Vicon data structure

    physics::JointPtr actuator;
    double virtual_time;

	physics::ModelPtr mod;

    //render_synchronization_buffer_c  rsync;

    //log_ptr log;                             //< state log
    wb_gazebo_session_ptr log;
  public:
    //-------------------------------------------------------------------------
    controller_c( void ) { }

    //-------------------------------------------------------------------------
    virtual ~controller_c( void ) {

      event::Events::DisconnectWorldUpdateBegin( _updateConnection );
    }

    //-------------------------------------------------------------------------
    // Gazebo callback.  Called when the simulation is starting up
    virtual void Load( physics::ModelPtr model, sdf::ElementPtr sdf ) {
      this->mod = model;
      std::string validation_errors;
      _world = world_ptr( new world_c( model->GetWorld() , model ));
      if( !_world->validate( validation_errors ) ) {
        printf( "Unable to validate world in controller\n%s\nERROR: Plugin failed to load\n", validation_errors.c_str() );
        return;
      }

      _world->reset();

      // get references to objects in the world
      _weazelball = _world->weazelball();
   
      // register update callback
      _updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind( &controller_c::Update, this ) );

      actuator = _weazelball->actuator();
     virtual_time = 0.0;   

/*
      int r = rsync.open();
      if( r != 0 ) {
        printf("ERROR: rsync open return code %d\n", r );
      }
*/

	  math::Pose initPose =  this->mod->GetRelativePose();
	  std::cout << "init pose = " << initPose << std::endl;
      log = wb_gazebo_session_ptr( new wb_gazebo_session_c(wb_gazebo_session_c::WRITE) );
      if( log->open("sim.log") ) {
        printf( "ERROR: unable to open gazebo log for writing\nsimulation controller failed to load\n" );
        exit(1);
        return;
      }
      // -- FIN --
      printf( "simulation controller has initialized\n" ); 
    }

    //-------------------------------------------------------------------------
    // Gazebo callback.  Called whenever the simulation advances a timestep
    virtual void Update( ) {
/*
      srand((unsigned)time(NULL));
	  math::Pose initPose =  this->mod->GetRelativePose();
	  math::Quaternion randomRotation = math::Quaternion( ((double)rand()/(double)3.14159), ((double)rand()/(double)3.14159),((double)rand()/(double)3.14159));
	  math::Pose newPose = math::Pose(initPose.pos, randomRotation);
	  std::cout << "New Pose = " << newPose << std::endl;
	  this->mod->SetRelativePose(newPose);
*/
      //rsync.synchronize( );
      double t = _world->sim_time();

      double dt = 0.00001; //This relates to how the world file is set up look for the variable with "1000"

      double motor_freq = 7.5;
      double theta_0 = 0;
      double theta_t = actuator->GetAngle( 0 ).Radian();
      double omega = actuator->GetVelocity( 0 );

      double desired_omega = 2.0 * PI * motor_freq;
      double desired_theta_t = t * desired_omega + theta_0;

      double Kp = 0.3;
      double Kd = 0.03;

      double u = Kp * (desired_theta_t - theta_t) + Kd * (desired_omega - omega)
;
      if( u > 1e16 ) exit( 0 );

      //printf( "%f: %f\n", t, u );
      //double u = 0.001;

      actuator->SetForce( 0, u );

      virtual_time += dt;

      log->write( t, _weazelball );
      //rsync.yield_to_render();
    }
  };

  GZ_REGISTER_MODEL_PLUGIN( controller_c )

} // namespace gazebo

//-----------------------------------------------------------------------------

