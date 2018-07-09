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
#include <string>

#include <tuple>
#define PI 3.14159265359
const int NUMBER_OF_WEASELBALLS = 4;
const std::string NAME_OF_WEASELBALLS = "swarmbot";
const std::string SHELL_STRING = "shell";
const std::string COLLECTION_PATH = "/home/justin/Documents/bouncy/self-assembly/gazebo/src/data/collections/collection.csv";
namespace gazebo
{
    struct weaselballData
	{
		math::Vector3 position;
		math::Vector3 linearVelocity;
		math::Vector3 linearAcceleration;
		math::Vector3 rotationalVelocity;
		math::Vector3 rotationalAcceleration;
		math::Vector3 rotationalDisplacement;//Yaw-Pitch-Roll

	};
}
namespace gazebo 
{
  class StateCollector : public WorldPlugin
  {
  private:
    event::ConnectionPtr _updateConnection;  //< Gazebo update callback

    physics::WorldPtr _world;                        //< Gazebo world pointer
	std::vector<std::vector<weaselballData>> collections; //A collection is a vector of a vector that has n elements of weaselballData where n is the number of weaselballs
	std::vector<physics::ModelPtr> weaselballs;
	bool getModelsFlag = 1;
	std::ofstream collectionFile;
	
  public:
    //-------------------------------------------------------------------------
    StateCollector( void ) { }
    virtual ~StateCollector( void ) {

      event::Events::DisconnectWorldUpdateBegin( _updateConnection );
	  this->collectionFile.close();
    }

	std::vector<physics::ModelPtr> getWeaselballs()
	{
		std::vector<physics::ModelPtr> ret;
		std::vector<physics::ModelPtr> allModels = this->_world->GetModels();
		for(auto it : allModels)
		{
			std::string name = it->GetName();
			if(name.find(NAME_OF_WEASELBALLS) != std::string::npos)
			{
				ret.push_back(it);
			}
		} 
		return ret;

	}
	/*
    struct weaselballData
	{
		math::Vector3 position;
		math::Vector3 linearVelocity;
		math::Vector3 linearAcceleration;
		math::Vector3 rotationalVelocity;
		math::Vector3 rotationalAcceleration;
		math::Vector3 rotationalDisplacement;//Yaw-Pitch-Roll

	};*/
	void writeDataToCSV(weaselballData data)
	{
		//Create the header of the csv and init
		this->collectionFile << data.position[0] << ", " << data.position[1] << ", " << data.position[2] << ", \n";	
	}
	

	
	bool checkAllWeaselsInit(std::vector<physics::ModelPtr> weaselballs)
	{
		return (weaselballs.size() == NUMBER_OF_WEASELBALLS ? 1 : 0);
	}

    //-------------------------------------------------------------------------
    // Gazebo callback.  Called when the simulation is starting up
    virtual void Load( physics::WorldPtr world, sdf::ElementPtr sdf ) {
	  this->_world = world;
      this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind( &StateCollector::Update, this ) );

	  this->collectionFile.open (COLLECTION_PATH,std::ofstream::out);
	  this->collectionFile << "Pos_x, Pos_y, Pos_z, LinearVelocity_X, LinearVelocity_Y, LinearVelocity_Z, LinearAcceleration_X, LinearAcceleration_Y, LinearAcceleration_Z, RotationalVelocity_X, RotationalVelocity_Y, RotationalVelocity_Z, RotationalAcceleration_X, RotationalAcceleration_Y, RotationalAcceleration_Z, Yaw, Pitch, Roll\n"; 
	  std::cout << "State Collector has loaded!" << std::endl;
    }

    //-------------------------------------------------------------------------
    // Gazebo callback.  Called whenever the simulation advances a timestep
    virtual void Update( ) {
		if(this->getModelsFlag)
		{
			std::vector<physics::ModelPtr> weaselballs = getWeaselballs();
			if( checkAllWeaselsInit(weaselballs) )
			{
				this->getModelsFlag = 0;
				this->weaselballs = weaselballs;
			}
			else
			{
				return;
			}
		}
		
		std::vector<weaselballData> collection;
		for (auto weaselball : this->weaselballs)
		{
			weaselballData data;
			//Collect data for current state of robot i
			physics::JointPtr shell = weaselball->GetJoint(SHELL_STRING);
			getXYZCoordinates(weaselball, &data);
			getLinearVelocity(weaselball, &data);
			getAngularVelocity(weaselball, &data);
			getAngularAcceleration(weaselball,  &data);

			writeDataToCSV(data);
		    collection.push_back(data);	
		}
		this->collections.push_back(collection);

    }

	void getXYZCoordinates(physics::ModelPtr weaselball, weaselballData* data)
	{
		math::Pose worldPose = weaselball->GetWorldPose();
		math::Vector3 pos(0,0,0);
		pos = worldPose.pos;
		data->position = pos;

	}

	void getLinearVelocity(physics::ModelPtr weaselball, weaselballData* data)
	{
		data->linearVelocity = weaselball->GetWorldLinearVel();
	}
	
	void getAngularAcceleration(physics::ModelPtr weaselball, weaselballData* data)
	{
		data->rotationalAcceleration = weaselball->GetWorldAngularAccel();
	}

	void getAngularVelocity(physics::ModelPtr weaselball, weaselballData* data)
	{
		data->rotationalVelocity = weaselball->GetWorldAngularVel();
	}

	void getLinearAcceleration(physics::ModelPtr weaselball, weaselballData* data)
	{
		data->linearAcceleration = weaselball->GetWorldLinearAccel();
	}

  };

  GZ_REGISTER_WORLD_PLUGIN( StateCollector )

} // namespace gazebo

//-----------------------------------------------------------------------------

