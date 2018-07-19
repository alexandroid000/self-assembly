#include "../include/bumpSensor.h"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(BumpSensor)

/////////////////////////////////////////////////
BumpSensor::BumpSensor() : SensorPlugin()
{
}

/////////////////////////////////////////////////
BumpSensor::~BumpSensor()
{
}

/////////////////////////////////////////////////
void BumpSensor::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
	std::cout << "ContactPlugin requires a contactSensor.\n" << std::endl;
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&BumpSensor::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
  std::cout << "Contact Sensor Succesfully Initialzied" << std::endl;
}

/////////////////////////////////////////////////
void BumpSensor::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
	//If it is a swarmbot or the ground ignore it
	std::string model1Name = contacts.contact(i).collision1();
	std::string model2Name = contacts.contact(i).collision2();
	if(model1Name.find("swarmbot") != std::string::npos or model1Name.find("myGroundPlane") != std::string::npos)
	{
		return;
	}
	if(model2Name.find("swarmbot") != std::string::npos or model2Name.find("myGroundPlane") != std::string::npos)
	{
		return;
	}
	//[MyGroundPlane::link::collision]
	//[swarmbot0::shell::base_geom_collision]
    	
    std::cout << "Collision between[" << model1Name
              << "] and [" << model2Name << "]\n";

	

  }
}
