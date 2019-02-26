#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <ros/console.h>
#include "../include/gazebo_log.h"
#include "../include/common.h"
#include "../include/helper.h"

#include <stdlib.h>
#include <vector>

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <time.h>       /* time */
#include <cmath>
#include <cfenv>

namespace gazebo
{

	class InitCondition : public WorldPlugin
	{
		private:

		bool resetFlag_;
		bool resetStructure_;
        bool resetBalls_;
		physics::WorldPtr world_;
		event::ConnectionPtr _updateConnection;
		event::ConnectionPtr _updateWorldReset;
		int cycleCounter_ = 0;
		std::vector<math::Vector3> positions_{math::Vector3(0,0,0), math::Vector3(0,0.385,0), math::Vector3(0.385,0.385,0)};
		bool doneInitFlag = 0;
		

		public:

        InitCondition() : WorldPlugin()
        {
			
        }

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
			this->world_ = _world;
            this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind( &InitCondition::Update, this ) );
			this->resetBalls_ = RANDOMIZE_BALLS;
			this->resetStructure_ = RANDOMIZE_STRUCTURES;
			this->resetFlag_ = 1;
			srand (static_cast <unsigned> (time(0)));	
			//Randomize the models that need to be randomized
			this->world_->SetPaused(0);	
			
			std::cout << "Finished loading in initial state of balls" << std::endl;
        }

		void worldReset()
		{
			this->resetFlag_ = 1;
		}

		void Update()
		{
			//Check that everything has initialized
            if (!this->doneInitFlag)
			{
				if(checkAllModelsInit())
				{
					this->doneInitFlag = 1;
					this->_updateWorldReset = event::Events::ConnectWorldReset(boost::bind( &InitCondition::worldReset, this));
                    this->resetFlag_ = 1; //Reset world at start to randomize if needed
				}
			}
			//Check that trials dont need a reset
			if(RUN_TRIALS)
			{
				if( (int)(this->cycleCounter_ / 2 ) == NUMBER_OF_TRIALS_CYCLES)
				{
					std::cout << "[Debug] Ending Simulator" << std::endl;
					std::cout << "[Debug] cycleCounter = "<< this->cycleCounter_ << " NUMBER_OF_TRIALS_CYCLES = " << NUMBER_OF_TRIALS_CYCLES << std::endl; 
					system("killall -9 gzclient & killall -9 gzserver & killall -9 rosmaster");
					this->world_->Fini();	 
				}
				if(this->world_->GetSimTime().sec > INDIVIDUAL_TRIAL_TIME)
				{
					ROS_INFO("Reseting World to change robot position!");
					this->world_->Reset();

				}
			}
			if(this->resetFlag_)
			{
					//Pause the world
					this->world_->SetPaused(1);
					//Randomize the models that need to be randomized
					cycleTest();
					randomizeStructures();
					randomizeBalls();
					//Unpause the world
					this->world_->SetPaused(0);	
					this->resetFlag_ = 0;
			}
		}
		//This function is used in cycle test to be able to 
	    math::Vector3 getNewPos()
		{
			//Use Maximum length of weaselball structure to figure out spawn box in the enclosure that wont cause the weaselball structure to spawn on the walls.
			//The corners of the inner enclosure are at (0.52, 0.52) (-0.52, 0.52) (-0.52, -0.52) (0.52, -0.52)
			double offset = LONGEST_WEASELBALL_SEQUENCE*DIAMETER_OF_WEASELBALLS;
			offset /= 2; //Divide by 2 because normally gazebo will have the center of the longest part be the center of the model (This is approximate true, it uses average volume)
			double max_offset = 0.52-offset;
			double dist = max_offset * 2;  
			double increments = dist / 5.0; //Breaking up the x and y's into 5 by 5 grids
			
			double new_x = (this->cycleCounter_ / 5) % 5;
			double new_y = (this->cycleCounter_ % 5);
			
			math::Vector3 pos(-max_offset + new_x*increments,-max_offset + new_y*increments,0);
			return pos;


		}
		void cycleTest()
		{
		//The point of cycle test is to move the weaselball configuration into meaningful areas to collect better data for the markov chain
//  ---------------------
//  |x   x    x   x   x |
//  |                   |
//  |x   x    x   x   x |
//  |                   |
//  |x   x    x   x   x |
//  |                   |
//  |x   x    x   x   x |
//  |                   |
//  |x   x    x   x   x |
//  ---------------------
// The x's in the above show the approximate spots where the weaselball structure will start
// Due to symmetry of the of the board the robot is in this is all of the areas we need to test
// There 3 states for testing in this case are S = {Near No walls, Near 1 wall, Near 2 walls (corner) }
// These points are figured out by hand for now. But a geometry based approach could figure out where to logically place the structures
			
			if(!RUN_TRIALS)
				return;

            std::vector<physics::ModelPtr> structures = getModels(NAME_OF_MOUNTS);
			for (auto it : structures)
			{
					math::Pose worldPoseWeaselball = it->GetWorldPose();
					math::Vector3 rotation(0,0,0);
					rotation = worldPoseWeaselball.rot.GetAsEuler();
					
					//Get New Position.
					math::Vector3 pos = getNewPos();
					
					math::Pose newPose(pos, rotation);
					it->SetWorldPose(newPose);
			}
			//Use the mountlinks to get the center position of the mounts
			//Move Ball to new position
			std::vector<physics::ModelPtr> weaselballs = getModels(NAME_OF_WEASELBALLS);
			for (auto it : structures)
			{
				int counter = 0;
				for (auto link : it->GetLinks())
			    {
					physics::ModelPtr weaselball = weaselballs[counter];
	
				    math::Pose worldPoseLink = link->GetWorldPose();	

					math::Vector3 pos(100,0,0);
					pos = worldPoseLink.pos;
	
					math::Pose worldPoseWeaselball = weaselball->GetWorldPose();
					math::Vector3 rotation(0,0,0);
					rotation = worldPoseWeaselball.rot.GetAsEuler();
						
					math::Pose newPose(pos, rotation);
					weaselball->SetWorldPose(newPose);
					counter += 1;
				}		 
			}
			this->cycleCounter_ += 1;
		}

		void randomizeStructures()
		{
			if(!this->resetStructure_)
				return;

            std::vector<physics::ModelPtr> structures = getModels(NAME_OF_MOUNTS);
			std::vector<physics::ModelPtr> weaselballs = getModels(NAME_OF_WEASELBALLS);
			//Rotate the structure
			for (auto it : structures)
			{
				math::Pose worldPose = it->GetWorldPose();
				math::Vector3 pos(0,0,0);
				pos = worldPose.pos;
				
				math::Vector3 rot(0,0, std::fmod(static_cast<float> (rand()), (float) 2*PI));
					
				math::Pose newPose(pos, rot);
				it->SetWorldPose(newPose);
			}
			//Use the mountlinks to get the center position of the mounts
			//Move Ball to new position
			for (auto it : structures)
			{
				int counter = 0;
				for (auto link : it->GetLinks())
			    {
					physics::ModelPtr weaselball = weaselballs[counter];
	
				    math::Pose worldPoseLink = link->GetWorldPose();	

					math::Vector3 pos(100,0,0);
					pos = worldPoseLink.pos;
	
					math::Pose worldPoseWeaselball = weaselball->GetWorldPose();
					math::Vector3 rotation(0,0,0);
					rotation = worldPoseWeaselball.rot.GetAsEuler();
						
					math::Pose newPose(pos, rotation);
					weaselball->SetWorldPose(newPose);
					counter += 1;
				}		 
			}
			
		}
 
		void randomizeBalls()
		{

			//First make sure we even want to reset the balls
			if(!this->resetBalls_)
				return;

			std::vector<physics::ModelPtr> weaselballs = getModels(NAME_OF_WEASELBALLS);
			//Randomize the balls
			for(auto it : weaselballs)
			{
				math::Pose worldPose = it->GetWorldPose();
				math::Vector3 pos(0,0,0);
				pos = worldPose.pos;
				
				math::Vector3 rot(std::fmod(static_cast<float> (rand()), (float) 2*PI), std::fmod(static_cast<float> (rand()), (float) 2*PI), std::fmod(static_cast<float> (rand()), (float) 2*PI));
					
				math::Pose newPose(pos, rot);
				it->SetWorldPose(newPose);
			
			}				
		}

		bool checkAllModelsInit()
		{
			std::vector<physics::ModelPtr> weaselballs = getModels(NAME_OF_WEASELBALLS);
            std::vector<physics::ModelPtr> structures = getModels(NAME_OF_MOUNTS);

            int number_of_weaselballs = 0;
            if(LARGE_ROBOT_GENERATOR)
            {
                number_of_weaselballs = K_LARGE;
            }
            else
            {
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
            }
			return ((weaselballs.size() == number_of_weaselballs and structures.size() == NUMBER_OF_STRUCTURES) ? 1 : 0);
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



    };
    GZ_REGISTER_WORLD_PLUGIN(InitCondition)
}

