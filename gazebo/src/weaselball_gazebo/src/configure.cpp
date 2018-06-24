#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

//http://gazebosim.org/tutorials?tut=plugins_world&cat=write_plugin
namespace gazebo
{
class Configure : public WorldPlugin
{
private:
  bool flag;
  event::ConnectionPtr _updateConnection;  //< Gazebo update callback

public:
  Configure() : WorldPlugin()
  {
  }

  virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load configure plugin.");
      return;
    }
    //register update callback
    _updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind( &Configure::Update, this ) );
    flag = 0;
    ROS_INFO("Configure Loaded!");
  }
  
	virtual void Update()
	{
		if(flag == 0)
		{
	   		createModel();
			flag = 1;
		}
	}

	int createModel()
	{
		ROS_INFO("CREATING MODEL");
	}
  

  

};
GZ_REGISTER_WORLD_PLUGIN(Configure)
}
