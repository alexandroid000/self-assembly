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
  class InitCondition : public WorldPlugin
  {
    public: InitCondition() : WorldPlugin()
            {
              printf("Hello World!\n");
            }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
            }
  };
  GZ_REGISTER_WORLD_PLUGIN(InitCondition)
}
