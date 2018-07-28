#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include "../include/gazebo_log.h"
#include "../include/common.h"

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

		public:

        InitCondition() : WorldPlugin()
        {
			
        }

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
			this->world_ = _world;
            this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind( &InitCondition::Update, this ) );
			this->_updateWorldReset = event::Events::ConnectWorldReset(boost::bind( &InitCondition::worldReset, this));
			this->resetBalls_ = RANDOMIZE_BALLS;
			this->resetStructure_ = RANDOMIZE_STRUCTURES;
			this->resetFlag_ = 1;
			srand (static_cast <unsigned> (time(0)));	
			std::cout << "Finished loading in initial state of balls" << std::endl;
        }

		void worldReset()
		{
			this->resetFlag_ = 1;
		}

		void Update()
		{
			if(this->resetFlag_)
			{
				if(checkAllModelsInit())
					//Reset the world
					this->world_->Reset();
					//Pause the world
					this->world_->SetPaused(1);
					//Randomize the models that need to be randomized
					randomizeStructures();
					randomizeBalls();
					//Unpause the world
			//		this->world_->SetPaused(0);	
					this->resetFlag_ = 0;
			}
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
			    std::cout << "NEW STRUCT ROT" << rot << std::endl; 	
					
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
			return ((weaselballs.size() == NUMBER_OF_WEASELBALLS and structures.size() == NUMBER_OF_STRUCTURES) ? 1 : 0);
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

