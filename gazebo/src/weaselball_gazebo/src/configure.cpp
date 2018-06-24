#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
class Factory : public WorldPlugin
{
  private:
      bool flag = 0;
      event::ConnectionPtr _updateConnection;  //< Gazebo update callback

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    _updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind( &Factory::Update, this ) );


    // Option 3: Insert model from file via message passing.

      // Create a new transport node
      transport::NodePtr node(new transport::Node());

      // Initialize the node with the world name
      node->Init(_parent->GetName());

      // Create a publisher on the ~/factory topic
      transport::PublisherPtr factoryPub =
      node->Advertise<msgs::Factory>("~/factory");

      // Create the message
      msgs::Factory msg;

      // Model file to load
      msg.set_sdf_filename("model://mount");

      // Pose to initialize the model to
      msgs::Set(msg.mutable_pose(),
          ignition::math::Pose3d(
            ignition::math::Vector3d(1, -2, 0),
            ignition::math::Quaterniond(0, 0, 0)));

      // Send the message
      factoryPub->Publish(msg);

  }
  Factory() : WorldPlugin()
  {
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
      // Create the message
        ROS_INFO("CREATING MODEL");
    }

};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Factory)
}
