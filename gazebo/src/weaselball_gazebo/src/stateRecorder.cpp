#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include "../include/gazebo_log.h"

#include <stdlib.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include <tuple>
#define PI 3.14159265359
const int NUMBER_OF_WEASELBALLS = 4;

namespace gazebo
{
    struct weaselballData
	{
		std::tuple <double,double,double> position;
		std::tuple <double,double,double> linearVelocity;
		std::tuple <double,double,double> linearAcceleration;
		std::tuple <double,double,double> rotationalVelocity;
		std::tuple <double,double,double> rotationalAcceleration;

	};
}
namespace gazebo 
{
  class StateCollector : public WorldPlugin
  {
  private:
    event::ConnectionPtr _updateConnection;  //< Gazebo update callback

    world_ptr _world;                        //< Gazebo world pointer
	std::vector<std::vector<weaselballData>> collections; //A collection is a vector of a vector that has n elements of weaselballData where n is the number of weaselballs
	std::vector<physics::ModelPtr> weaselballs;
	
  public:
    //-------------------------------------------------------------------------
    StateCollector( void ) { }
    virtual ~StateCollector( void ) {

      event::Events::DisconnectWorldUpdateBegin( _updateConnection );
    }

	std::vector<physics:ModelPtr> getWeaselballs()
	{


	}

    //-------------------------------------------------------------------------
    // Gazebo callback.  Called when the simulation is starting up
    virtual void Load( physics::WorldPtr world, sdf::ElementPtr sdf ) {
	  this->_world = world;
      this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind( &StateCollector::Update, this ) );
	  this->weaselballs = getWeaselballs();

	  std::cout << "State Collector has loaded!" << std::endl;
    }

    //-------------------------------------------------------------------------
    // Gazebo callback.  Called whenever the simulation advances a timestep
    virtual void Update( ) {
		std::vector<weaselballData> collection;
		for (int i = 0; i < NUMBER_OF_WEASELBALLS; i++)
		{
			weaselballData data;
			//Collect data for current state of robot i
			//
		    collection.push_back(data);	
		}
		this->collections.push_back(collection);

    }

	void getXYZCoordinates(physics::ModelPtr weaselball, weaselballData* data)
	{

	}

	void getLinearVelocity(physics::ModelPtr weaselball, weaselballData* data)
	{

	}
	
	void getAngularVelocity(physics::ModelPtr weaselball, weaselballData* data)
	{

	}

	void getLinearAcceleration(physics::ModelPtr weaselball, weaselballData* data)
	{

	}

  };

  GZ_REGISTER_WORLD_PLUGIN( StateCollector )

} // namespace gazebo

//-----------------------------------------------------------------------------

