#define DEBUG 1  //Set to 1 to print debug to serial, set to 0 to run normally
//if 0, info topic will be advertised, an messages printed to that.
#define ID "R1" //MODULE ID, should be unique for each ESP8266 chip to ensure topics are unique.  
#define PRINTDEBUG(STR) \
  {  \
    if (DEBUG) Serial.println(STR); \ 
  }
#include <ros.h>
#include <ESP8266WiFi.h>
#include <std_msgs/String.h>
#include "ESP8266Hardware.h"
#include "ros/node_handle.h"

ros::NodeHandle_<ESP8266Hardware> nh;

long spinTime = 0;//time of last spin
long pubTime = 0;

char* ssid     = "IllinoisNet_Guest";
char* password = "";
char* host = "172.16.187.111";
byte mac[6];


std_msgs::String string_msg;


// declare publishers
String myName = String("ESP_")+String(ID);
ros::Publisher pub(myName.c_str(), &string_msg);

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
    WiFi.macAddress(mac);
    Serial.print("MAC: ");
    Serial.print(mac[5],HEX);
    Serial.print(":");
    Serial.print(mac[4],HEX);
    Serial.print(":");
    Serial.print(mac[3],HEX);
    Serial.print(":");
    Serial.print(mac[2],HEX);
    Serial.print(":");
    Serial.print(mac[1],HEX);
    Serial.print(":");
    Serial.println(mac[0],HEX);
  }
}

void setup() {
  Serial.begin(9600);  //Start Serial
  delay(10);
  Serial.println("setup begin");
  connectWifi(ssid, password); // Start WiFi
  // put your setup code here, to run once:
  nh.initNode(host);
  nh.advertise(pub);

  delay(10);

  //spin the node once before starting
  nh.spinOnce();
}

void loop() {

   if(DEBUG){
      string_msg.data = "debug in loop";
      pub.publish( &string_msg );
      PRINTDEBUG("LOOP");
      nh.spinOnce();
   }
   else{
       //Publish at fixed intervals to prevent comm from blocking serial reading
       //TODO: fix this, maybe go to interrupt model for serial?
       long curTime = millis(); 
       if(curTime-pubTime>50){
           pub.publish( &string_msg );
           pubTime = curTime;  
       }
       curTime = millis();
       if(curTime-spinTime>2000){
           nh.spinOnce();
           spinTime = curTime;
       }
   }
}
