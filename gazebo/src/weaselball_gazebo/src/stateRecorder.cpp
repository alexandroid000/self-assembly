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
		math::Vector3 rotationalDisplacement;//Yaw-Pitch-Roll
		math::Vector3 linearVelocityWorld;
		math::Vector3 linearAccelerationWorld;
		math::Vector3 rotationalVelocityWorld;
		math::Vector3 rotationalAccelerationWorld;
		math::Vector3 linearVelocityRelative;
		math::Vector3 linearAccelerationRelative;
		math::Vector3 rotationalVelocityRelative;
		math::Vector3 rotationalAccelerationRelative;

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
	void writeDataToCSV(weaselballData data, int id)
	{
		this->collectionFile << id << ", ";
		//Create the header of the csv and init
		this->collectionFile << data.position[0] << ", " << data.position[1] << ", " << data.position[2] << ", ";	
		this->collectionFile << data.rotationalDisplacement[1] << ", " << data.rotationalDisplacement[1] << ", " << data.rotationalDisplacement[2] << ", ";	

		this->collectionFile << data.linearVelocityWorld[0] << ", " << data.linearVelocityWorld[1] << ", " << data.linearVelocityWorld[2] << ", ";	
		this->collectionFile << data.linearAccelerationWorld[0] << ", " << data.linearAccelerationWorld[1] << ", " << data.linearAccelerationWorld[2] << ", ";	
		this->collectionFile << data.rotationalVelocityWorld[0] << ", " << data.rotationalVelocityWorld[1] << ", " << data.rotationalVelocityWorld[2] << ", ";	
		this->collectionFile << data.rotationalAccelerationWorld[0] << ", " << data.rotationalAccelerationWorld[1] << ", " << data.rotationalAccelerationWorld[2] << ", ";	

		this->collectionFile << data.linearVelocityRelative[0] << ", " << data.linearVelocityRelative[1] << ", " << data.linearVelocityRelative[2] << ", ";	
		this->collectionFile << data.linearAccelerationRelative[0] << ", " << data.linearAccelerationRelative[1] << ", " << data.linearAccelerationRelative[2] << ", ";	
		this->collectionFile << data.rotationalVelocityRelative[0] << ", " << data.rotationalVelocityRelative[1] << ", " << data.rotationalVelocityRelative[2] << ", ";	
		this->collectionFile << data.rotationalAccelerationRelative[0] << ", " << data.rotationalAccelerationRelative[1] << ", " << data.rotationalAccelerationRelative[2] << ", \n";	
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
	  this->collectionFile << "ID, Pos_x, Pos_y, Pos_z, Yaw, Pitch, Roll, Linear_Velocity_X_World, Linear_Velocity_Y_World, Linear_Velocity_Z_World, Linear_Acceleration_X_World, Linear_Acceleration_Y_World, Linear_Acceleration_Z_World, Rotational_Velocity_X_World, Rotational_Velocity_Y_World, Rotational_Velocity_Z_World, Rotational_Acceleration_X_World, Rotational_Acceleration_Y_World, Rotational_Acceleration_Z_World,Linear_Velocity_X_Relative, Linear_Velocity_Y_Relative, Linear_Velocity_Z_Relative, Linear_Acceleration_X_Relative, Linear_Acceleration_Y_Relative, Linear_Acceleration_Z_Relative_Relative, Rotational_Velocity_X_Relative, Rotational_Velocity_Y_Relative, Rotational_Velocity_Z_Relative, Rotational_Acceleration_X_Relative, Rotational_Acceleration_Y_Relative, Rotational_Acceleration_Z_Relative \n"; 

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
			std::string name = weaselball->GetName();
			//Collect data for current state of robot i
			physics::JointPtr shell = weaselball->GetJoint(SHELL_STRING);

			getXYZCoordinatesWorld(weaselball, &data);
			getRotationalDisplacementWorld(weaselball, &data);
			
			getLinearVelocityWorld(weaselball, &data);
			getAngularVelocityWorld(weaselball, &data);
			getAngularAccelerationWorld(weaselball,  &data);
			getLinearAccelerationWorld(weaselball, &data);

			getLinearVelocityRelative(weaselball, &data);
			getAngularVelocityRelative(weaselball, &data);
			getAngularAccelerationRelative(weaselball,  &data);
			getLinearAccelerationRelative(weaselball, &data);

			writeDataToCSV(data, name.back());
		    collection.push_back(data);	
		}
		this->collections.push_back(collection);

    }

	void getXYZCoordinatesWorld(physics::ModelPtr weaselball, weaselballData* data)
	{
		math::Pose worldPose = weaselball->GetWorldPose();
		math::Vector3 pos(0,0,0);
		pos = worldPose.pos;
		data->position = pos;

	}

	void getRotationalDisplacementWorld(physics::ModelPtr weaselball, weaselballData* data)
	{
		math::Pose worldPose = weaselball->GetWorldPose();
		math::Vector3 rotation(0,0,0);
		rotation = worldPose.rot.GetAsEuler();
		data->rotationalDisplacement = rotation;

	}

	void getLinearVelocityWorld(physics::ModelPtr weaselball, weaselballData* data)
	{
		data->linearVelocityWorld = weaselball->GetWorldLinearVel();
	}
	
	void getAngularAccelerationWorld(physics::ModelPtr weaselball, weaselballData* data)
	{
		data->rotationalAccelerationWorld = weaselball->GetWorldAngularAccel();
	}

	void getAngularVelocityWorld(physics::ModelPtr weaselball, weaselballData* data)
	{
		data->rotationalVelocityWorld = weaselball->GetWorldAngularVel();
	}

	void getLinearAccelerationWorld(physics::ModelPtr weaselball, weaselballData* data)
	{
		data->linearAccelerationWorld = weaselball->GetWorldLinearAccel();
	}

	void getLinearVelocityRelative(physics::ModelPtr weaselball, weaselballData* data)
	{
		data->linearVelocityRelative = weaselball->GetRelativeLinearVel();
	}
	
	void getAngularAccelerationRelative(physics::ModelPtr weaselball, weaselballData* data)
	{
		data->rotationalAccelerationRelative = weaselball->GetRelativeAngularAccel();
	}

	void getAngularVelocityRelative(physics::ModelPtr weaselball, weaselballData* data)
	{
		data->rotationalVelocityRelative = weaselball->GetRelativeAngularVel();
	}

	void getLinearAccelerationRelative(physics::ModelPtr weaselball, weaselballData* data)
	{
		data->linearAccelerationRelative = weaselball->GetRelativeLinearAccel();
	}
  };

  GZ_REGISTER_WORLD_PLUGIN( StateCollector )

} // namespace gazebo

//-----------------------------------------------------------------------------

