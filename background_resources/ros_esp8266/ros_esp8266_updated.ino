#define DEBUG 1  //Set to 1 to print debug to serial, set to 0 to run normally
//if 1, info topic will be advertised, an messages printed to that.
#define ID "a1" //MODULE ID, should be unique for each ESP8266 chip to ensure topics are unique.  

#include <ros.h>
#include <ESP8266WiFi.h>
#include <std_msgs/String.h>
#include "ESP8266Hardware.h"
#include "ros/node_handle.h"
#include <ESP8266Ping.h>

//Networking information
char* ssid     = "IllinoisNet_Guest";
char* password = "";
char* host = "192.17.237.1"; //changed 7/12/17 when updated software on host pc, was 172.16.187.111
int   port = 8009;
WiFiClient client;
IPAddress server(192,17,237,1);

//ROS system info
long spinTime = 0;//time of last spin
long pubTime = 0;
ros::NodeHandle_<ESP8266Hardware> nh;
std_msgs::String string_msg;

//declaring ESP8266 as publisher
String myName = String("ESP_") + String(ID);
ros::Publisher pub(myName.c_str(), &string_msg);
int poseFlag = 0;

//Function for connecting the ESP to wifi
//Additionally will attempt to connect to a host PC at specified IP and socket
void connectWifi(const char* ssid, const char* password, int socket)
{
  WiFi.disconnect();
  int timeOutCounter = 0;
  int timeOut = 30;
  Serial.print("Attempting connection to:");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && timeOutCounter < timeOut) {
    delay(1000);
    timeOutCounter++;  
    Serial.print(".");
    };
  if (WiFi.status() == WL_CONNECTED)
  {
    if (!(Ping.ping("www.google.com")))
    {
      Serial.println("Ping test failed. Exiting connection");
      return;
    }
    Serial.println("Connection Successful");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Attempting to connect to host PC at: ");
    Serial.print(host);
    Serial.print(" through socket: ");
    Serial.println(socket);
    if (Ping.ping(server))
    {
      client.connect(server, socket);
      if (client.read() == -1) //-1 indicates failure to connect to host PC on specified socket
      {
       Serial.println("Error in connecting to socket, read failure");
      }
      else Serial.println("Connection to host successful");
    }
    else
    {
      Serial.println("Failed to ping host computer");
    }
  }
  else{
    Serial.println("Failed to connect to Wifi");
  }
   
}

void setup() {
  Serial.begin(9600);  //Start Serial
  delay(10);
  connectWifi(ssid, password, port);
  // put your setup code here, to run once:
  nh.initNode(host);
  nh.advertise(pub);
  delay(10);

  //spin the node once before starting
  nh.spinOnce();
}

void loop()
{
}


