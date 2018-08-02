
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo-7/gazebo/sensors/SensorManager.hh>

#include "../include/gazebo_log.h"

#include <ctime>
#include <stdlib.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <tuple>
#include "../include/common.h"

//This data structures is used to store information about the weaselballs/mount. 
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
		
		math::Vector3 mountPosition;
		math::Vector3 mountRotation;
		
		common::Time timeStamp;

	};
}

namespace gazebo 
{
  class StateCollector : public WorldPlugin
  {
  private:
    event::ConnectionPtr _updateConnection;  //< Gazebo update callback
    event::ConnectionPtr _updateWorldReset;  //< Gazebo update callback

    physics::WorldPtr _world;                        //< Gazebo world pointer
	std::vector<physics::ModelPtr> weaselballs;

	std::vector<physics::ModelPtr> structures;
	bool getModelsFlag = 1;
	//recordingType is set to 0 to collect a lot of data about the weaselballs and is set to 1 to collect R2 x S1 of the mount configuration.
	int recordingType_ = 1;
	std::ofstream collectionFile;
	int resetCounter = 0;
	
  public:
    //-------------------------------------------------------------------------
    StateCollector( void ) { }
    virtual ~StateCollector( void ) {

      event::Events::DisconnectWorldUpdateBegin( _updateConnection );
	  this->collectionFile.close();
    }

	std::vector<physics::ModelPtr> getModels(std::string modelName)
	{
		std::vector<physics::ModelPtr> ret;
		std::vector<physics::ModelPtr> allModels = this->_world->GetModels();
		for(auto it : allModels)
		{
			std::string name = it->GetName();
			if(name.find(modelName) != std::string::npos)
			{
				ret.push_back(it);
			}
		} 
		return ret;

	}



	void writeDataToCSV(weaselballData data,int id)
	{
		if(this->recordingType_ == 0)
		{
			this->collectionFile << data.timeStamp << ",";
			this->collectionFile << id << ",";
			this->collectionFile << data.mountPosition[0] << "," << data.mountPosition[1] << ",";
			//Gazebo does rotation as rpy
			this->collectionFile << data.mountRotation[2] << ",";
		//Create the header of the csv and init
			this->collectionFile << data.position[0] << "," << data.position[1] << "," << data.position[2] << ",";	
			this->collectionFile << data.rotationalDisplacement[1] << "," << data.rotationalDisplacement[1] << "," << data.rotationalDisplacement[2] << ",";	

			this->collectionFile << data.linearVelocityWorld[0] << "," << data.linearVelocityWorld[1] << "," << data.linearVelocityWorld[2] << ",";	
			this->collectionFile << data.linearAccelerationWorld[0] << "," << data.linearAccelerationWorld[1] << "," << data.linearAccelerationWorld[2] << ",";	
			this->collectionFile << data.rotationalVelocityWorld[0] << "," << data.rotationalVelocityWorld[1] << "," << data.rotationalVelocityWorld[2] << ",";	
			this->collectionFile << data.rotationalAccelerationWorld[0] << "," << data.rotationalAccelerationWorld[1] << "," << data.rotationalAccelerationWorld[2] << ",";	

			this->collectionFile << data.linearVelocityRelative[0] << "," << data.linearVelocityRelative[1] << "," << data.linearVelocityRelative[2] << ",";	
			this->collectionFile << data.linearAccelerationRelative[0] << "," << data.linearAccelerationRelative[1] << "," << data.linearAccelerationRelative[2] << ",";	
			this->collectionFile << data.rotationalVelocityRelative[0] << "," << data.rotationalVelocityRelative[1] << "," << data.rotationalVelocityRelative[2] << ",";	
			this->collectionFile << data.rotationalAccelerationRelative[0] << "," << data.rotationalAccelerationRelative[1] << "," << data.rotationalAccelerationRelative[2] << ",";	
			this->collectionFile << this->resetCounter << ",";
			this->collectionFile << checkCorrectness() << ",";
			this->collectionFile << "\n";
		}
		else if(this->recordingType_ == 1)
		{
			this->collectionFile << data.timeStamp << ",";
			this->collectionFile << id << ",";
			this->collectionFile << data.mountPosition[0] << "," << data.mountPosition[1] << ",";
			//Gazebo does rotation as rpy
			this->collectionFile << data.mountRotation[2] << ",";
			this->collectionFile << this->resetCounter << ",";
			this->collectionFile << checkCorrectness() << ",";
			this->collectionFile << "\n";
		}
	}

	//The purpose of the function is to check that everything at a high level looks like it is working well in the simulator	
	bool checkCorrectness()
	{
	//Get Max distance between a structure
	//Make sure none of the balls are over the Max distance away from eachother + lil extra (say 1.2 times)
		return 1;

	}
	
	bool checkAllModelsInit(std::vector<physics::ModelPtr> weaselballs, std::vector<physics::ModelPtr> structure)
	{
		return ((weaselballs.size() == NUMBER_OF_WEASELBALLS and structure.size() == NUMBER_OF_STRUCTURES) ? 1 : 0);
	}

	void worldReset()
	{
	    this->resetCounter++;
	}
	
	//This function checks if there are any collisions with the wall
	int getNumberOfWalls()
	{
		//Get weaselballs bumpsensor
		gazebo::sensors::SensorManager* mgr = sensors::SensorManager::Instance();
//		sensorsList = sensorManager.GetSensors();
//		std::cout << "Sensor name is " << sensorList[0]->GetName() << std::endl;	
		
	
	}

    //-------------------------------------------------------------------------
    // Gazebo callback.  Called when the simulation is starting up
    virtual void Load( physics::WorldPtr world, sdf::ElementPtr sdf ) {
	  this->_world = world;
      this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind( &StateCollector::Update, this ) );
      this->_updateWorldReset = event::Events::ConnectWorldReset(
        boost::bind( &StateCollector::worldReset, this ) );
	  //Get time and append to string name
	  time_t rawtime;
	  struct tm * timeinfo;
	  char buffer[80];

	  time (&rawtime);
	  timeinfo = localtime(&rawtime);

	  strftime(buffer,sizeof(buffer),"%m-%d-%Y_%H-%M-%S",timeinfo);
	  std::string str(buffer);

	  std::stringstream ss;
	  if(recordingType_ == 0)
	  {
          ss << COLLECTION_PATH << str << "_long.csv";

      }
	  else if(recordingType_ == 1)
      {
          ss << COLLECTION_PATH << str << ".csv";
      }

	  this->collectionFile.open (ss.str(),std::ofstream::out);
	  if(this->recordingType_ == 0)
	  {
	  this->collectionFile << "Time,ID,Mount_X,Mount_Y,Mount_Yaw,Pos_x,Pos_y,Pos_z,Yaw,Pitch,Roll,Linear_Velocity_X_World,Linear_Velocity_Y_World,Linear_Velocity_Z_World,Linear_Acceleration_X_World,Linear_Acceleration_Y_World,Linear_Acceleration_Z_World,Rotational_Velocity_X_World,Rotational_Velocity_Y_World,Rotational_Velocity_Z_World,Rotational_Acceleration_X_World,Rotational_Acceleration_Y_World,Rotational_Acceleration_Z_World,Linear_Velocity_X_Relative,Linear_Velocity_Y_Relative,Linear_Velocity_Z_Relative,Linear_Acceleration_X_Relative,Linear_Acceleration_Y_Relative,Linear_Acceleration_Z_Relative_Relative,Rotational_Velocity_X_Relative,Rotational_Velocity_Y_Relative,Rotational_Velocity_Z_Relative,Rotational_Acceleration_X_Relative,Rotational_Acceleration_Y_Relative,Rotational_Acceleration_Z_Relative,ResetID,checkCorrectness\n"; 
	  }
		else if(recordingType_ == 1)
		{
			this->collectionFile << "Time,ID,X,Y,Yaw,ResetID,checkCorrectness\n";

		}

	//Get structure object and store it
	
	  std::cout << "State Collector has loaded!" << std::endl;
    }

    //-------------------------------------------------------------------------
    // Gazebo callback.  Called whenever the simulation advances a timestep
    virtual void Update( ) {
		//Wait for all models to spawn
		if(this->getModelsFlag)
		{
			std::vector<physics::ModelPtr> weaselballs = getModels(NAME_OF_WEASELBALLS);
			std::vector<physics::ModelPtr> structures = getModels(NAME_OF_MOUNTS);
			if( checkAllModelsInit(weaselballs, structures) )
			{
				this->getModelsFlag = 0;
				this->weaselballs = weaselballs;
				this->structures = structures;
			}
			else
			{
				return;
			}
		}
		//Collect the data for this update if everything is spawned
		std::vector<weaselballData> collection;
	    if(recordingType_ == 0)
		{
			weaselballData data;
			for (auto mount : this->structures)
			{

				getMountXYZ(mount, &data);
				getMountRotation(mount, &data);
			    getSimTime(&data);

			}
			for (auto weaselball : this->weaselballs)
			{
				std::string name = weaselball->GetName();

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
			    getSimTime(&data);

				writeDataToCSV(data, name.back());
				collection.push_back(data);	
			}
		}
		else if(this->recordingType_ == 1 )
		{
			for (auto mount : this->structures)
			{
				weaselballData data;

				getMountXYZ(mount, &data);
				getMountRotation(mount, &data);
			    getSimTime(&data);

				writeDataToCSV(data,0); //I will worry about creating IDs for substructures later...
				collection.push_back(data);	 
			}

		}
		
    }
	
	void getSimTime(weaselballData* data)
	{
		data->timeStamp = this->_world->GetSimTime();
	}

	void getMountXYZ(physics::ModelPtr mount, weaselballData* data)
	{
		math::Pose worldPose = mount->GetWorldPose();
		math::Vector3 pos(0,0,0);
		pos = worldPose.pos;
		data->mountPosition = pos;
	}

	void getMountRotation(physics::ModelPtr mount, weaselballData* data)
	{
		math::Pose worldPose = mount->GetWorldPose();
		math::Vector3 rotation(0,0,0);
		rotation = worldPose.rot.GetAsEuler();
		data->mountRotation = rotation; 
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

