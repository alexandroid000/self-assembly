#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include "common.h"

namespace gazebo
{
 // typedef boost::shared_ptr<Sensor> SensorPtr;
 // typedef boost::shared_ptr<ContactSensor> ContactSensorPtr;
  /// \brief An example plugin for a contact sensor.
  class BumpSensor : public SensorPlugin
  {
    /// \brief Constructor.
    public: BumpSensor();

	public: std::string parseForModelName();
    /// \brief Destructor.
    public: virtual ~BumpSensor();
	public: std::vector<physics::JointPtr> joints_;
	bool jointsFlag_ = 0;
	std::ofstream collectionFile;

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the contact sensor's update signal.
    private: virtual void OnWorldUpdate();
	int countFOO = 0;	
	void writeToCSV();
	int getNumberOfWalls();
	common::Time getSimTime();
	void detatch();

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;

  };
}
#endif
