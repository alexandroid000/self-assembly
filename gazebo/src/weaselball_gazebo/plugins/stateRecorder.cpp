
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo-7/gazebo/sensors/SensorManager.hh>

#include "../include/gazebo_log.h"
#include <typeinfo>
#include <ros/ros.h>
#include <ros/console.h>
#include <ctime>
#include <stdlib.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>

#include <tuple>
#include "../include/common.h" //Used for static const global variables
#include "../include/helper.h" //is_in

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

		int numberOfWalls;
        std::vector<std::string> wallIDs;


	};
}

namespace gazebo 
{
  class StateCollector : public WorldPlugin
  {
  private:
    event::ConnectionPtr _updateConnection;  //< Gazebo update callback
    event::ConnectionPtr _updateWorldReset;  //< Gazebo update callback
    event::ConnectionPtr _updateCollision;  //< Gazebo update callback

    physics::WorldPtr world_;                        //< Gazebo world pointer
	std::vector<physics::ModelPtr> weaselballs;
	std::vector<sensors::ContactSensorPtr> bumpSensor;
	std::vector<physics::ModelPtr> structures;
	bool getModelsFlag = 1;
	//recordingType is set to 0 to collect a lot of data about the weaselballs and is set to 1 to collect R2 x S1 of the mount configuration.
	std::ofstream collectionFile;
	int resetCounter = 0;
	
	int wallCounter = 0;
	std::vector<std::string> railStrings;
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
		std::vector<physics::ModelPtr> allModels = this->world_->GetModels();
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
		if(RECORDING_TYPE == 0)
		{
			this->collectionFile << data.timeStamp << ",";
			this->collectionFile << id << ",";
			this->collectionFile << data.numberOfWalls << ",";
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
			this->collectionFile << data.numberOfWalls << ",";
			this->collectionFile << "\n";
		}
		else if(RECORDING_TYPE == 1)
		{
			this->collectionFile << data.timeStamp << ",";
			this->collectionFile << id << ",";
			this->collectionFile << data.mountPosition[0] << "," << data.mountPosition[1] << ",";
			//Gazebo does rotation as rpy
			this->collectionFile << data.mountRotation[2] << ",";
			this->collectionFile << this->resetCounter << ",";
			this->collectionFile << checkCorrectness() << ",";
			this->collectionFile << data.numberOfWalls << ",";
            for (auto i = data.wallIDs.begin(); i != data.wallIDs.end(); ++i)
            {
                std::string tempstring = (*i);
                std::string railNumber = tempstring.substr(tempstring.find("rail") + 4, 2); //Get the id of the rail        
                int railNumberInt = std::stoi(railNumber);
                this->collectionFile << railNumberInt;
                //If it isn't the last element in the vecotr, add an &
                if (std::next(i) != data.wallIDs.end())
                {
                    this->collectionFile << "&";
                }
            }
			this->collectionFile << "\n";
		}
	}

	//The purpose of the function is to check that everything at a high level looks like it is working well in the simulator	
	bool checkCorrectness()
	{
		//The system is correct if all of the balls are in a hub and are within the enclosure
		std::vector<physics::ModelPtr> weaselballs = getModels(NAME_OF_WEASELBALLS);
		std::vector<physics::ModelPtr> structures = getModels(NAME_OF_MOUNTS);
		double MOUNT_RADIUS = 0.0504;
		double ENCLOSURE_MAX_X = 0.52;
		double ENCLOSURE_MAX_Y = 0.52;
		double ENCLOSURE_MIN_X = -0.52;
		double ENCLOSURE_MIN_Y = -0.52;
		bool valid = 1;
		for (auto it : weaselballs)
		{
			math::Vector3 ballPose = it->GetWorldPose().pos;
			bool flag = 0;
			//Make sure the balls are in a hub
			for (auto structure : structures)
			{
				std::vector<physics::LinkPtr> links = structure->GetLinks();
				for (auto link : links)
				{
					math::Vector3 linkPose = link->GetWorldPose().pos;
					//Distance between a given hub and the weaselballs
					double foo = pow(linkPose[0]-ballPose[0],2) + pow(linkPose[1]-ballPose[1],2);
					if (pow(foo,0.5) < MOUNT_RADIUS)
					{
						flag = 1;
						break;
					}
				}	
				if(flag)
					break;
			}
			valid = flag;	
			if(!valid)
			{
				ROS_INFO("Balls not in hub!");
				break;
			}

			//Make sure ball is within the enclosure
			if(ballPose[0] < ENCLOSURE_MIN_X || ballPose[0] > ENCLOSURE_MAX_X || ballPose[1] < ENCLOSURE_MIN_Y || ballPose[1] > ENCLOSURE_MAX_Y)	
			{
				ROS_INFO("Balls not in enclosure!");
				valid = 0;
				break;
			}	

		}
		if(!valid)
		{
			ROS_INFO("NOT VALID!");
			this->world_->Reset();
			
		}
		return valid;

	}
	
	bool checkAllModelsInit(std::vector<physics::ModelPtr> weaselballs, std::vector<physics::ModelPtr> structure)
	{

        int number_of_weaselballs = 0; //We will infer the number of weaselballs from the inputted structure instead of asking the user to ask for it.
        if(is_in(ROBOT_TO_RUN,{1}))
            number_of_weaselballs = 1;
        else if (is_in(ROBOT_TO_RUN,{2}))
            number_of_weaselballs = 2;
        else if (is_in(ROBOT_TO_RUN,{3,4,5}))
            number_of_weaselballs = 3;
        else if (is_in(ROBOT_TO_RUN,{6,7,8,9,10}))
            number_of_weaselballs = 4;
        else
            ROS_ERROR("[stateRecorder.cpp/checkAllModels] Robot to run should be in the range of 1 to 10");
        ROS_INFO("Number of weaselballs is %d", number_of_weaselballs);
		return ((weaselballs.size() == number_of_weaselballs and structure.size() == NUMBER_OF_STRUCTURES) ? 1 : 0);
	}

	void worldReset()
	{
	    this->resetCounter++;
	}
	
	std::vector<sensors::ContactSensorPtr> getContactSensor()
	{
		gazebo::sensors::SensorManager* mgr = sensors::SensorManager::Instance();
		std::vector<sensors::SensorPtr> sensorsList = mgr->GetSensors();
		std::vector<sensors::ContactSensorPtr> ret;
		for(auto it : sensorsList)
		{
			ret.push_back(std::dynamic_pointer_cast<sensors::ContactSensor>(it));
		}
        ROS_INFO("Found %d bump sensors", ret.size());
		return ret;
	}


    //-------------------------------------------------------------------------
    // Gazebo callback.  Called when the simulation is starting up
    virtual void Load( physics::WorldPtr world, sdf::ElementPtr sdf ) {
	  this->world_ = world;
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
	  if(RECORDING_TYPE == 0)
	  {
          ss << COLLECTION_PATH << str << "_long.csv";

      }
	  else if(RECORDING_TYPE == 1)
      {
          ss << COLLECTION_PATH << str << ".csv";
      }

	  this->collectionFile.open (ss.str(),std::ofstream::out);
	  if(RECORDING_TYPE == 0)
	  {
	  this->collectionFile << "Time,ID,Mount_X,Mount_Y,Mount_Yaw,Pos_x,Pos_y,Pos_z,Yaw,Pitch,Roll,Linear_Velocity_X_World,Linear_Velocity_Y_World,Linear_Velocity_Z_World,Linear_Acceleration_X_World,Linear_Acceleration_Y_World,Linear_Acceleration_Z_World,Rotational_Velocity_X_World,Rotational_Velocity_Y_World,Rotational_Velocity_Z_World,Rotational_Acceleration_X_World,Rotational_Acceleration_Y_World,Rotational_Acceleration_Z_World,Linear_Velocity_X_Relative,Linear_Velocity_Y_Relative,Linear_Velocity_Z_Relative,Linear_Acceleration_X_Relative,Linear_Acceleration_Y_Relative,Linear_Acceleration_Z_Relative_Relative,Rotational_Velocity_X_Relative,Rotational_Velocity_Y_Relative,Rotational_Velocity_Z_Relative,Rotational_Acceleration_X_Relative,Rotational_Acceleration_Y_Relative,Rotational_Acceleration_Z_Relative,ResetID,checkCorrectness\n"; 
	  }
		else if(RECORDING_TYPE == 1)
		{
			this->collectionFile << "Time,ID,X,Y,Yaw,ResetID,checkCorrectness,NumberOfWalls,WallId(s)\n";

		}

	//Get structure object and store it
	
	  ROS_INFO("State Collector has loaded!");
    }
	void onCollision()
	{
		msgs::Contacts contacts;
		for(auto contactSensor : this->bumpSensor)
		{
			contacts = contactSensor->GetContacts();
			for (unsigned int i = 0; i < contacts.contact_size(); ++i)
			{
				//If it is a weaselball or the ground ignore it
				std::string model1Name = contacts.contact(i).collision1();
				std::string model2Name = contacts.contact(i).collision2();

				std::map<std::string, physics::Contact> mapping = contactSensor->Contacts(model1Name);
				physics::ModelPtr mount;
				std::string linkNumber;
				
				if(model1Name.find("mount") != std::string::npos and model2Name.find("rail") != std::string::npos)
				{

					//Check that rail# isn't already counted
					if (std::find(this->railStrings.begin(), this->railStrings.end(), model2Name) == this->railStrings.end())
					{
						
					  // Element not in vector.
						this->wallCounter += 1;
						this->railStrings.push_back(model2Name);
					}
					else
					{
					}
				}
				else if(model1Name.find("rail") != std::string::npos and model2Name.find("mount") != std::string::npos)
				{
					ROS_INFO("BAR");
					//Check that rail# isn't already counted
					if (std::find(this->railStrings.begin(), this->railStrings.end(), model1Name) == this->railStrings.end())
					{
					  // Element not in vector.
						this->wallCounter += 1;
						this->railStrings.push_back(model1Name);
					}
					else
					{

					}
				}

			}
		}

	}
    //-------------------------------------------------------------------------
    // Gazebo callback.  Called whenever the simulation advances a timestep
    virtual void Update( ) {
		//Wait for all models to spawn
		if(this->getModelsFlag)
		{
			std::vector<physics::ModelPtr> weaselballs = getModels(NAME_OF_WEASELBALLS);
			std::vector<physics::ModelPtr> structures = getModels(NAME_OF_MOUNTS);
			std::vector<sensors::ContactSensorPtr> bumpSensor = getContactSensor();
			if( checkAllModelsInit(weaselballs, structures) and bumpSensor.size() != 0)
			{
				ROS_INFO("Found all the models and Sensors!");
				this->getModelsFlag = 0;
				this->weaselballs = weaselballs;
				this->structures = structures;
				this->bumpSensor = bumpSensor;
				for (auto it : bumpSensor)
				{
					it->Init();
					it->SetActive(1);
					this->_updateCollision = it->ConnectUpdated(
						std::bind(&StateCollector::onCollision,  this));
					ROS_INFO("Sensor status = %d", it->IsActive());

				}
			}
			else
			{
                ROS_WARN("Waiting for all models and Sensors to spawn");
                ROS_WARN("Models init = %d", checkAllModelsInit(weaselballs, structures));
                ROS_WARN("Bump Sensor init = %d", bumpSensor.size());
				return;
			}
		}
		//Collect the data for this update if everything is spawned
		std::vector<weaselballData> collection;
	    if(RECORDING_TYPE == 0)
		{
			weaselballData data;
			for (auto mount : this->structures)
			{

				getMountXYZ(mount, &data);
				getMountRotation(mount, &data);
			    getSimTime(&data);
				getNumberOfWalls(&data);
                getRailStrings(&data);

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
		else if(RECORDING_TYPE == 1 )
		{
			for (auto mount : this->structures)
			{
				weaselballData data;

				getMountXYZ(mount, &data);
				getMountRotation(mount, &data);
			    getSimTime(&data);
				getNumberOfWalls(&data);
                getRailStrings(&data);

				writeDataToCSV(data,0); //I will worry about creating IDs for substructures later...
				collection.push_back(data);	 
			}

		}
		
    }
	
	void getSimTime(weaselballData* data)
	{
		data->timeStamp = this->world_->GetSimTime();
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

	//This function checks if there are any collisions with the wall
	void getNumberOfWalls(weaselballData* data)
	{
		data->numberOfWalls = this->wallCounter;
		this->wallCounter = 0;
	}

    void getRailStrings(weaselballData* data)
    {
        //Do a deep copy.
        data->wallIDs = this->railStrings;       
		this->railStrings.clear();
    }

  };

  GZ_REGISTER_WORLD_PLUGIN( StateCollector )

} // namespace gazebo

//-----------------------------------------------------------------------------

