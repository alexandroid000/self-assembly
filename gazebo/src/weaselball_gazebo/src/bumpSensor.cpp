#include "../include/bumpSensor.h"
#include <regex>
#include <unistd.h>

using namespace gazebo;
using namespace std;
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

	std::map<std::string, physics::Contact> mapping = this->parentSensor->Contacts(model1Name);
	physics::ModelPtr mount;
	std::string linkNumber;
	//Ignore collisions unless it is "coll" and "rail"
	if(model1Name.find("mount") != std::string::npos and model2Name.find("rail") != std::string::npos)
	{
		for (const auto &p : mapping) {
			if(model1Name.find("mount") != std::string::npos)
			{
				std::cout << "M = " << p.second.collision1->GetName();
				mount = p.second.collision1->GetModel();
				std::string coll = p.second.collision1->GetName();
				linkNumber = coll.substr(4, coll.length());
			}
			else
			{
				std::cout << "M = " << p.second.collision2->GetName();
				mount = p.second.collision2->GetModel();
				std::string coll = p.second.collision2->GetName();
				linkNumber = coll.substr(4, coll.length());
			}

		}

	}
	else if(model1Name.find("rail") != std::string::npos and model2Name.find("mount") != std::string::npos)
	{
		for (const auto &p : mapping) {
			if(model1Name.find("coll") != std::string::npos)
			{
				std::cout << "M = " << p.second.collision1->GetName();
				mount = p.second.collision1->GetModel();
				std::string coll = p.second.collision1->GetName();
				linkNumber = coll.substr(4, coll.length());
			}
			else
			{
				std::cout << "M = " << p.second.collision2->GetName();
				mount = p.second.collision2->GetModel();
				std::string coll = p.second.collision2->GetName();
				linkNumber = coll.substr(4, coll.length());
			}
		}

	}
	else
	{
		continue;
	}


	std::vector<physics::LinkPtr> links = mount->GetLinks();
	std::vector<physics::JointPtr> joints = mount->GetJoints();
	if(this->jointsFlag == 0)
	{
		this->joints_ = joints;
		this->jointsFlag = 1;
	}

	for (auto it : links)
	{
		std::cout << "Link = " << it->GetName() << std::endl;
	}	
	for (auto it : joints)
	{
		std::cout << "Joint = " << it->GetName() << std::endl;
//		it->Detach();
	}	


	for (auto it : links)
	{
		std::cout << "ID is = " << linkNumber << "Link is = " << it->GetName() << std::endl;
		if(it->GetName().find(linkNumber) != std::string::npos)
		{
			std::cout << "MATCH FOUND" << std::endl;
			for (int i = 0; i < this->joints_.size() ; i++)
			{
				auto j = this->joints_[i]; 
				try
				{
						if(j->GetParent()->GetName() == it->GetName())
						{
							std::cout << "PARENT = " << j->GetParent()->GetName() << " LINK = " << it->GetName() << std::endl;
							std::cout << "DETACHING " << j->GetName() << std::endl;
							this->joints_.erase(this->joints_.begin() + i);
							j->Detach();
							j->Fini();
							
						}
						if(j->GetChild()->GetName() == it->GetName())
						{
							std::cout << "CHILD = " << j->GetChild()->GetName() << " LINK = " << it->GetName() << std::endl;
							std::cout << "DETACHING " << j->GetName() << std::endl;
							this->joints_.erase(this->joints_.begin() + i);
							j->Detach();
							j->Fini();
						}
				}
				catch (const std::exception& e)
				{
					std::cout << "CAUGHT" << std::endl;
				}
				catch (...)
				{
					std::cout << "Caught it" << std::endl;
				}
			}
			break;
		}
	}
			std::cout << "GOT TO END FLAG " << std::endl; 
	return;
  }
}
