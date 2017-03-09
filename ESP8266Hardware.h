#ifndef ROS_ESP8266_HARDWARE_H_
#define ROS_ESP8266_HARDWARE_H_

#if !defined(DEBUG)
  #define DEBUG 1  //Print to Serial
#endif

#define PRINTDEBUG(STR) \
  {  \
    if (DEBUG) Serial.println(STR); \
  }

#include <ESP8266WiFi.h>
#define SERIAL_CLASS WiFiClient
  #if !defined(ROSSERIAL_PORTNUM)
	#define ROSSERIAL_PORTNUM 11411					// default port number
#endif

class ESP8266Hardware {
  public:
    ESP8266Hardware(SERIAL_CLASS* io , long baud= 57600){
      iostream = io;
      baud_ = baud;
    }
    
    ESP8266Hardware()
    {
    //  iostream = &Serial;
      baud_ = 57600;
    }
    
    ESP8266Hardware(ESP8266Hardware& h){
      this->iostream = iostream;
      this->baud_ = h.baud_;
    }

    void setBaud(long baud){
      this->baud_= baud;
    }
  
    int getBaud(){return baud_;}
        
    void init(char* hostname) {
  	  iostream = new WiFiClient();
  	  strcpy(this->hostname, hostname);
  	  
  	  while (!iostream->connect(hostname, ROSSERIAL_PORTNUM)) {
    		for (int i = 0; i < 100; i++) {
    		  delay(50);
    		}
    		return;
    	  }
    }
    int read(){return iostream->read();};
    
    void write(uint8_t* data, int length){
      for(int i=0; i<length; i++)
        iostream->write(data[i]);
    }

    unsigned long time(){return millis();}

    bool isConnectionUp(){
  	  if(iostream->connected())
    		return true;
  	  else {
  	  	iostream->stop();
  		  return iostream->connect(this->hostname, ROSSERIAL_PORTNUM);
  	  }
    }

  protected:
    char hostname[256];
    SERIAL_CLASS* iostream;
    long baud_;
};

#endif
