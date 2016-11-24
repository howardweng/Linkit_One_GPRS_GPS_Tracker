/* Linkit One GPS + GPRS Tracker
 *  The code is for Linkit One board 
    
  FRRUT.COM is glad to provide this open source code and you are free to distrubute, and it also took reference from many open source projects and sample codes, 
  such as LASS  https://github.com/LinkItONEDevGroup/LASS  project, and Mediatek sample codes...    
   
  On top of it, I invested a lot of time on this code for perfection.
    
  We welcome any opportunity about web and IoT trainings, or BUSINESS OPPORTUNITIES.
  Please visit FRRUT.COM for more detail and 
  ----> https://www.frrut.com  果子創意
    @author   Howard Weng (FRRUT.COM)  howard.weng@gmail.com  
    @file     Linkit_GPS_GPRS
    @license  BSD (see license.txt)  
   
    @section  HISTORY
    v1.0  - First release

If hardware and MOQTT broker is correctly set up. you can expect to receive the data as following.  
Topic:  owntracks/frrut/gps1
Json payload: 

{
  "tid":"GG",
  "data" : "118A282DFC8A6400000000",
  "bat" : 100
}


basic spec. of this code for this device. 
A. GPRS + GPS. 
B. Battery operated, so power saving is important. 
C. Battery level information report to server and display on app and web (100%,66%,33%,0%).
D. Position report by two trigger points. 
   a. 3-axis sensor receive activation signal, report x mins/time.  ( can be adjusted )
   b. Regular reports if no activation from motion sensor. That means if the device is not moving, then it only report every x hours. ( can be adjusted )
    
 */

 
//************************* Please change the following 3 essential credentials  *********************************************

char mqttBroker[] = "recorder.99101c17.svc.dockerapp.io";  //Change Mosquitto Host Name here. 
int portNumber = 1883;       // Change the Mosquitto PORT Number here.

String DeviceID = "GP";       // Optional change, only TWO characters or numbers allowed. this ID will display in maps of Owntracks APP. 

//*****************************************************************************************************************************


//No need to change the following settings,if you followed instructions in NODE-Red setting. 

#define mqttuser "howard"   //use "admin" account for login user name.  
#define mqttpwd ""        //in our setting in Mosquitto, no password is needed. 

#define chPrefix "owntracks"  // Temperay topic prefix 
#define chFromDevice "/frrut" // Temperay topic usename, only use it once. 
#define chAnalog "/gps1"    // Temperay topic devicename, only use it once.If you have more than one device, give different deviceID here. 

// LinkIt One sketch for MQTT Demo

#define __LINKIT_ONE_DEBUG__
#include <LGPRS.h>
#include <LGPRSClient.h>
#include <stdlib.h>
#include <PubSubClient.h>

#define chToDevice "/tod"  //Not used in this case, reserved for future use to receive data back. 

//connect LED to digital pin2 definition

#define LED 2 


// Time definition

#include <LDateTime.h>
datetimeInfo t;
char bufft[256];
byte lora_trans[22];  // Not about Lora, I just share the same variable name for uploading data to MQTT broker. 

// Json format definition

#include <ArduinoJson.h>  //for Json file

// GPS definition

#include <LGPS.h>
gpsSentenceInfoStruct g_info;

//Battery definition

#include <LBattery.h>
char buffb[256];


//GPRS definition

LGPRSClient c;
PubSubClient client( c );

//Reporting data definition

unsigned long lastSend;
char GPS_formatted[130]; 
long MemoryMillis;
long longtimewaiting; 


// 3 Axis Accerator definition. 

#include <Wire.h>
#include <ADXL345.h>
ADXL345 accelerometer;





void getGPSData(gpsSentenceInfoStruct &g_info, char* GPS_formatted)
{
  processled();
  LGPS.powerOn();
  boolean GPS_fix = false;

  while (!GPS_fix)
  {
    LGPS.getData(&g_info);                                      //get the data from the GPS and store it in 'g_info'
    GPS_fix = printGPGGA((char*)g_info.GPGGA,GPS_formatted);    //printGPGGA returns TRUE if the GPGGA string returned confirms a GPS fix.
  
  }
  LGPS.powerOff();   
}

boolean printGPGGA(char* str, char* GPS_formatted)
{
  char SMScontent[160];
  char latitude[20];
  char lat_direction[1];
  char longitude[20];
  char lon_direction[1];
  char buf[20];
  char time[30];
  const char* p = str;
  p = nextToken(p, 0); // GGA
  p = nextToken(p, time); // Time
  p = nextToken(p, latitude); // Latitude
  p = nextToken(p, lat_direction); // N or S?
  p = nextToken(p, longitude); // Longitude
  p = nextToken(p, lon_direction); // E or W?
  p = nextToken(p, buf); // fix quality
  if (buf[0] == '1')
  {
    // GPS fix
    p = nextToken(p, buf); // number of satellites
   Serial.print("GPS is fixed:");
   Serial.print(buf); 
   Serial.println();  
   Serial.print(atoi(buf));                  // Printing Satellite number!! 
   Serial.println("found!");


   Serial.println( " " );
   Serial.print( " Status Code [ = " );
   Serial.print( client.state() );
   Serial.println( " ]" );

   
   strcpy(SMScontent, "GPS fixed, satellites found: ");
   strcat(SMScontent, buf);
    
    const int coord_size = 8;
    char lat_fixed[coord_size],lon_fixed[coord_size];
    convertCoords(latitude,longitude,lat_fixed, lon_fixed,coord_size);
    

    Serial.println(lat_fixed);                  // Late I need.
    Serial.println(lon_fixed);                  // Late I need.

float GPS_LAT_f = (float)atof(lat_fixed);
float GPS_LON_f = (float)atof(lon_fixed);
   
  GPS_LAT_f += 90;
  GPS_LON_f += 180;
  
  unsigned long GPS_LAT_i = GPS_LAT_f*10000;   
  unsigned long GPS_LON_i = GPS_LON_f*10000;   

//Seperate GPS_LAT_i into 3* 8 bits(HEX) 

lora_trans[12] = (GPS_LAT_i >> 16); 
lora_trans[13] = (GPS_LAT_i >> 8); 
lora_trans[14] = (GPS_LAT_i);
lora_trans[15] = (GPS_LON_i >> 16); 
lora_trans[16] = (GPS_LON_i >> 8);
lora_trans[17] = (GPS_LON_i);
lora_trans[18] = (LBattery.level());

      Serial.println(" InitLGPRS()");//
      InitLGPRS();//
     
      Serial.println("Reconnect");
      reconnect();
      
      Serial.println("Push to cloud");
      pushDataCloud();   


return true;    
}

if ((millis()-MemoryMillis)>80000)
    {

      nosignalled();
            
      Serial.println(" Only Connecting 80 seconds and wait for next time ");
      Serial.println(MemoryMillis);
      Serial.println(millis());
      MemoryMillis=millis();

      Serial.println(" InitLGPRS()");
      InitLGPRS();//
     
      Serial.println("Reconnect");
      reconnect();
      
      Serial.println("Push to cloud");
      pushDataCloud();   //
 
     
      return true;
     }

       
  else
  {
     
    Serial.println("GPS is not fixed yet. waiting.......");

  //  When GPS is searching, LED flashs as following. 

  digitalWrite(LED, HIGH);   // set the LED on
  delay(200);
  digitalWrite(LED, LOW);   // set the LED off
  delay(100);
  digitalWrite(LED, HIGH);   // set the LED on
  delay(1000);
  digitalWrite(LED, LOW);   // set the LED off
  delay(100);
    
    return false;    
  }     
   delay(5000); //orignal 1000
}




void InitLGPRS()
{
  processled();
  
 Serial.println("Connecting to GPRS");
  while (0 ==  LGPRS.attachGPRS()) {
    delay(1000);
  }
  Serial.println("Connected to GPRS");  
  delay(1000); //wait 1 sec.
}



//Connecting to Mosquitto broker.  

void reconnect() {
  processled();

 // Loop until reconnected

    Serial.println("..Connecting to MQTT broker...");

// Create a random client ID
    String clientId = "linkitClient-";
    clientId += String(random(0xffff), HEX) + String(random(0xffff), HEX);
     
     Serial.println( clientId.c_str() );

     Serial.println( " " );
     Serial.print( " Before Client Connect MQTT Status Code [ = " );
     Serial.print( client.state() );
     Serial.println( " ]" );

// Attempt to connect    
 
     if ( client.connect(clientId.c_str(),mqttuser,mqttpwd) ) {  // Better use some random name   
   
      Serial.println( "[Connection DONE]" );

     processled();
      
// Subscribe to topic "inTopic"

  //     char inTopics[100];
  //     (String(chPrefix)+chToDevice).toCharArray( inTopics, sizeof(inTopics)) ;
  //     Serial.print("Subscribed to ->");
  //     Serial.println(inTopics);
   //   client.subscribe(inTopics );

     Serial.println( " " );
     Serial.print( " After Client Connect MQTT Status Code  (0 is connected) [ = " );
     Serial.print( client.state() );
     Serial.println( " ]" );      
    }
    else {
      // Wait 1 seconds before retrying
      Serial.println( " Try again to connect MQTT" ); 
      delay( 1000 );
    }     
    if (client.connected())    
    client.loop();   
 }



void pushDataCloud(){

 char buff[150];  
 sprintf(buff, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X", lora_trans[12], lora_trans[13], \
 lora_trans[14], lora_trans[15], lora_trans[16], lora_trans[17], lora_trans[18], lora_trans[2], lora_trans[2],  \
 lora_trans[2], lora_trans[2]); 


 StaticJsonBuffer<200> jsonBuffer;
 JsonObject& root = jsonBuffer.createObject();
    root["tid"] = DeviceID;
    root["data"] = (buff);
    root["bat"] = (LBattery.level());

    PrintTime();

 root.printTo(Serial);  // Printing data to serial for debug.
 Serial.println(" ");

  // Send payload
  
  char buffer1[200];
  root.printTo(buffer1, sizeof(buffer1));
  
  //build channel name
  
  char topic[100];
  (String(chPrefix)+chFromDevice+chAnalog).toCharArray( topic, sizeof(topic));

 Serial.println( " Client connection Status before sending data to broker " );
 Serial.print( " Status Code [ = " );
 Serial.print( client.state() );
 Serial.println( " ]" );
 Serial.println("******************   Firing Data ***************");

  client.publish(topic, buffer1 );   
 
  ledlight();  


    Serial.println( " Disconnecting MQTT client connection " );
    Serial.print( " Status code [ = " );
    Serial.print( client.state() );
    Serial.println( " ]" );
 
    client.disconnect(); 

    Serial.print( " Status Code now [ = " );
    Serial.print( client.state() );
    Serial.println( " ]" );
    Serial.println( " -------------------Finish this loop, Wait for next trigger -----------------------" );    
  
}




// LED flashing behaviors.

//Sending data LED

void ledlight(){    
  Serial.println("FIRING DATA LED");   
  digitalWrite(LED, HIGH);   // set the LED on
    delay(200);
  digitalWrite(LED, LOW);   // set the LED off
    delay(200);
  digitalWrite(LED, HIGH);   // set the LED on
    delay(200);
  digitalWrite(LED, LOW);   // set the LED off
    delay(200);
  digitalWrite(LED, HIGH);   // set the LED on
    delay(200);
  digitalWrite(LED, LOW);   // set the LED off
    delay(200);
  digitalWrite(LED, HIGH);   // set the LED on
    delay(200);
  digitalWrite(LED, LOW);   // set the LED off
    delay(200);
  digitalWrite(LED, HIGH);   // set the LED on
  delay(5000);               // for 500ms
  digitalWrite(LED, LOW);   // set the LED off
}

//Triggering action LED

void triggerled(){   

   Serial.println("Trigger LED"); 
  digitalWrite(LED, HIGH);   // set the LED on
    delay(100);
  digitalWrite(LED, LOW);   // set the LED off
    delay(100);
  digitalWrite(LED, HIGH);   // set the LED on
    delay(100);
  digitalWrite(LED, LOW);   // set the LED off 
      delay(100);
    digitalWrite(LED, HIGH);   // set the LED on
    delay(100);
  digitalWrite(LED, LOW);   // set the LED off
    delay(100);
  digitalWrite(LED, HIGH);   // set the LED on
    delay(100);
  digitalWrite(LED, LOW);   // set the LED off   
}

// GPS position searching not found LED

void nosignalled(){    
  Serial.println("No Signal LED");  
  digitalWrite(LED, HIGH);   // set the LED on
    delay(2000);
  digitalWrite(LED, LOW);   // set the LED off
    delay(1000);
  digitalWrite(LED, HIGH);   // set the LED on
    delay(2000);
  digitalWrite(LED, LOW);   // set the LED off 
   delay(1000);  digitalWrite(LED, HIGH);   // set the LED on
  digitalWrite(LED, LOW);   // set the LED off
}


// Processing data LED

void processled(){      

  Serial.println("Processled");
  digitalWrite(LED, HIGH);   // set the LED on
    delay(50);
  digitalWrite(LED, LOW);   // set the LED off
    delay(50);
  digitalWrite(LED, HIGH);   // set the LED on
    delay(50);
  digitalWrite(LED, LOW);   // set the LED off   
}



// GPS Coordinates, no need to change. 

void convertCoords(const char* latitude, const char* longitude, char* lat_return, char* lon_return, int buff_length)
{
  char lat_deg[3];
  strncpy(lat_deg,latitude,2);      //extract the first 2 chars to get the latitudinal degrees
  lat_deg[2] = 0;                   //null terminate
  char lon_deg[4];
  strncpy(lon_deg,longitude,3);      //extract first 3 chars to get the longitudinal degrees
  lon_deg[3] = 0;                    //null terminate
  int lat_deg_int = arrayToInt(lat_deg);    //convert to integer from char array
  int lon_deg_int = arrayToInt(lon_deg);
  float latitude_float = arrayToFloat(latitude);      //convert the entire degrees-mins-secs coordinates into a float - this is for easier manipulation later
  float longitude_float = arrayToFloat(longitude);
  latitude_float = latitude_float - (lat_deg_int*100);      //remove the degrees part of the coordinates - so we are left with only minutes-seconds part of the coordinates
  longitude_float = longitude_float - (lon_deg_int*100);
   latitude_float /=60;                                    //convert minutes-seconds to decimal
  longitude_float/=60;
  latitude_float += lat_deg_int;                          //add back on the degrees part, so it is decimal degrees
  longitude_float+= lon_deg_int;
   snprintf(lat_return,buff_length,"%2.3f",latitude_float);    //format the coordinates nicey - no more than 3 decimal places
  snprintf(lon_return,buff_length,"%3.3f",longitude_float);
}


int arrayToInt(const char* char_array)
{
  int temp;
  sscanf(char_array,"%d",&temp);
  return temp;
}

float arrayToFloat(const char* char_array)
{
  float temp;
  sscanf(char_array, "%f", &temp);
  return temp;
}

const char *nextToken(const char* src, char* buf)
{
  int i = 0;
  while (src[i] != 0 && src[i] != ',')
  i++;
  if (buf)
  {
    strncpy(buf, src, i);
    buf[i] = 0;
  }
  if (src[i])
  i++;
  return src + i;
}



// 3axis function checksetup

void checkSetup()
{
  Serial.print("Activity Threshold = "); Serial.println(accelerometer.getActivityThreshold());
  Serial.print("Inactivity Threshold = "); Serial.println(accelerometer.getInactivityThreshold());
  Serial.print("Time Inactivity = "); Serial.println(accelerometer.getTimeInactivity());

  Serial.print("Look activity on axis = "); 
  if (accelerometer.getActivityX()) { Serial.print(" X "); }
  if (accelerometer.getActivityY()) { Serial.print(" Y "); }
  if (accelerometer.getActivityZ()) { Serial.print(" Z "); }
  Serial.println();

  Serial.print("Look inactivity on axis = "); 
  if (accelerometer.getInactivityX()) { Serial.print(" X "); }
  if (accelerometer.getInactivityY()) { Serial.print(" Y "); }
  if (accelerometer.getInactivityZ()) { Serial.print(" Z "); }
  Serial.println();  
}

// Time settings function

void PrintTime(){

LDateTime.getTime(&t);
sprintf(bufft, "%d:%d:%d", t.hour, t.min, t.sec);
Serial.print(" TIME: ");
Serial.println(bufft);

}


// MQTT callback function

void callback( char* topic, byte* payload, unsigned int length ) {
  Serial.print( "Recived message on Topic:" );
  Serial.print( topic );
  Serial.print( "    Message:");  
char payloadC[length];
  for (int i=0;i<length;i++) {
    payloadC[i]=(char)payload[i];
    Serial.print( (char)payload[i] );
  }
    Serial.println();
}







void setup()
{

   delay( 1000 );
   Serial.begin( 115200 );

   client.setServer( mqttBroker, portNumber );
   client.setCallback( callback );
   lastSend = 0;
   MemoryMillis = 0;

// Light LED Settings initial off

  pinMode(LED, OUTPUT);  
  digitalWrite(LED, LOW);   // set the LED off  

// 3 Axis Settings

 Serial.println("Initialize ADXL345");
  if (!accelerometer.begin())
  {
    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
    delay(500);
  }

  accelerometer.setActivityThreshold(5.0);    // Recommended 2 g
  accelerometer.setInactivityThreshold(5.0);  // Recommended 2 g

  accelerometer.setTimeInactivity(180);         // Recommended 5 s
  accelerometer.setActivityXYZ(1);         // Check activity on X,Y,Z-Axis  // Set activity detection only on X,Y,Z-Axis
  accelerometer.setInactivityXYZ(1);       // Check inactivity on X,Y,Z-Axis    // Set inactivity detection only on X,Y,Z-Axis
  accelerometer.useInterrupt(ADXL345_INT1);    // Select INT 1 for get activities

  // Check settings
  checkSetup();
    
}



void loop()
{

// Read values for activities
delay(50);  
Vector norm = accelerometer.readNormalize();
// Read activities
Activites activ = accelerometer.readActivites();


  if (activ.isActivity)
  {
    if( millis()-lastSend > 20000 ) { // Send an update only after 60 seconds
    lastSend = millis();     

    Serial.println(" ");
    Serial.println("++++++    ACTION   +++++++++");
    
     triggerled();  
       
     PrintTime();  

     getGPSData(g_info,GPS_formatted);
   
   
       
    }
  }


if ((millis()-longtimewaiting)>3600000)
    {
      Serial.println(longtimewaiting);
      Serial.println(millis());
      longtimewaiting=millis();
     
    Serial.println(" ");
    Serial.println(" ");
    Serial.println("********  NO ACTION  *********");
    Serial.println("********  Fire Every 60 MINS  *********");

       triggerled();
 
       PrintTime();

       getGPSData(g_info,GPS_formatted);
    
 
           
   }

  
}


