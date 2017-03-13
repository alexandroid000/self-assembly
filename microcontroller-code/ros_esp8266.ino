#define DEBUG 0  //Set to 1 to print debug to serial, set to 0 to run normally
//if 0, info topic will be advertised, an messages printed to that.
#define ID "R1" //MODULE ID, should be unique for each ESP8266 chip to ensure topics are unique.  
#define PRINTDEBUG(STR) \
  {  \
    if (DEBUG) Serial.println(STR); \ 
  }
#include <ESP8266WiFi.h>
#include <ros.h>
#include <geometry_msgs/Pose.h>
#include "ESP8266Hardware.h"
#include "ros/node_handle.h"

ros::NodeHandle_<ESP8266Hardware> nh;

long spinTime = 0;//time of last spin
long pubTime = 0;

char* ssid     = "";
char* password = "";
char* host = "192.168.";


// POSE MSG
geometry_msgs::Pose pose_msg;


// declare publishers
String poseName = String("Pose_")+String(ID);
ros::Publisher pose(poseName.c_str(), &pose_msg);

//Flags for publishing
int poseFlag = 0;

// Function to connect WiFi
void connectWifi(const char* ssid, const char* password) {
  int WiFiCounter = 0;
  // We start by connecting to a WiFi network
  if(DEBUG){PRINTDEBUG("Connecting to ");}
  if(DEBUG){PRINTDEBUG(ssid);}
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && WiFiCounter < 30) {
    delay(1000);
    WiFiCounter++;
    if(DEBUG){PRINTDEBUG(".")};
  }

  if(DEBUG){
    PRINTDEBUG("");
    PRINTDEBUG("WiFi connected");
    PRINTDEBUG("IP address: ");
    PRINTDEBUG(WiFi.localIP());
  }
}

void setup() {
  Serial.begin(115200);  //Start Serial
  delay(10);
  connectWifi(ssid, password); // Start WiFi
  // put your setup code here, to run once:
  nh.initNode(host);
  nh.advertise(pose);

  delay(10);

  //spin the node once before starting
  nh.spinOnce();
}

void loop() {

   if(DEBUG){
      pose_msg.orientation.x = 0.0;
      pose_msg.orientation.y = 1.0;
      pose_msg.orientation.z = 2.0;
      pose_msg.orientation.w = 3.0;
      pose.publish( &pose_msg );
      PRINTDEBUG("LOOP");
      nh.spinOnce();
   }
   else{
       //Publish at fixed intervals to prevent comm from blocking serial reading
       //TODO: fix this, maybe go to interrupt model for serial?
       long curTime = millis(); 
       if(curTime-pubTime>50){
           if(poseFlag == 1){
              pose.publish( &pose_msg );
              poseFlag = 0;
           }
           pubTime = curTime;  
       }
       curTime = millis();
       if(curTime-spinTime>2000){
           nh.spinOnce();
           spinTime = curTime;
       }
   }
}
