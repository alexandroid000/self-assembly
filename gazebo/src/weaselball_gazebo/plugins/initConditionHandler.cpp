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

        bool resetBalls_ = 0;
		physics::WorldPtr world_;
		event::ConnectionPtr _updateConnection;

		public:

        InitCondition() : WorldPlugin()
        {
			
        }

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
			this->world_ = _world;
            this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind( &InitCondition::Update, this ) );
			this->resetBalls_ = RANDOMIZE_BALLS;
			srand (static_cast <unsigned> (time(0)));	
			std::cout << "Finished loading in initial state of balls" << std::endl;
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

		virtual void Update()
		{
			//First make sure we even want to reset the balls
			if(!this->resetBalls_)
				return;

			//Check that everything has spawned
			std::vector<physics::ModelPtr> weaselballs = getModels(NAME_OF_WEASELBALLS);
            std::vector<physics::ModelPtr> structures = getModels(NAME_OF_MOUNTS);
		    if(checkAllModelsInit(weaselballs,structures))
			{
				//Reset the world
				this->world_->Reset();
				//Pause the world
				this->world_->SetPaused(1);
				//Randomize the balls
				for(auto it : weaselballs)
				{
					math::Pose worldPose = it->GetWorldPose();
					math::Vector3 pos(0,0,0);
					pos = worldPose.pos;
					
					math::Vector3 rot(std::fmod(static_cast<float> (rand()), (float) PI), std::fmod(static_cast<float> (rand()), (float) PI), std::fmod(static_cast<float> (rand()), (float) PI));
					std::cout << "New rotation is " << rot << std::endl;
				    	
					math::Pose newPose(pos, rot);
					it->SetWorldPose(newPose);
				}				
				//Unpause the world
			    this->world_->SetPaused(0);	
				this->resetBalls_ = 0;
			}	

		}

		bool checkAllModelsInit(std::vector<physics::ModelPtr> weaselballs, std::vector<physics::ModelPtr> structure)
		{
			return ((weaselballs.size() == NUMBER_OF_WEASELBALLS and structure.size() == NUMBER_OF_STRUCTURES) ? 1 : 0);
		}



    };
    GZ_REGISTER_WORLD_PLUGIN(InitCondition)
}

