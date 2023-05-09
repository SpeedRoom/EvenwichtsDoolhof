
//Autor: Emiel Dever

#include <AccelStepper.h>
#include <WiFi.h>
#include "OTAlib.h"
#include <PubSubClient.h>


//structuur van pasword and ssid aangepast
//define and const int aangepast
//bal einde tesen
//zorgen dat de mqtt server aanligt


























//MQTT -
#define SSID          "NETGEAR68"
#define PWD           "excitedtuba713"
#define MQTT_SERVER   "192.168.1.61"  
#define MQTT_PORT     1883
#define topic  "esp_doolhof/output"

WiFiClient espClient;
PubSubClient client(espClient);
void setup_wifi()
{
  delay(10);
  Serial.println("Connecting to WiFi..");
  WiFi.begin(SSID, PWD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


//OTA
OTAlib ota(SSID, PWD);

Pin connections
#define finish 12
#define dirPiny 18
#define stepPiny 19
#define dirPinx 33
#define stepPinx 32
#define switchy 14
#define switchx 27
#define poty 39
#define potx 35
//when using pcb version 1.2 use these pins instead of the ones above
//#define potx 34
//#define poty 35

// const int finish = 12;
// const int dirPiny = 18;
// const int stepPiny = 19;
// const int dirPinx = 33;
// const int stepPinx = 32;
// const int switchy = 14;
// const int switchx = 27;
// //const int poty = 26;
// //const int potx = 25;
// const int poty = 39;
// const int potx = 35;
// //when using pcb version 1.2 use these pins instead of the ones above
// //const int potx = 34;
// //const int poty = 35;

int posy = 0;
int value_poty = 0;
int posx = 0;
int value_potx = 0;
const int max_roty = 1920;
const int max_rotx = 2720;


void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // creat unique client ID
    // in Mosquitto broker enable anom. access
    if (client.connect("ESP32Client"))
    {
      Serial.println("connected");
      // Subscribe
      client.subscribe(topic);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 1 second");
      vTaskDelay(1000/portTICK_RATE_MS);
    }
    taskYIELD();
  }
}

AccelStepper mySteppery(AccelStepper::DRIVER, stepPiny, dirPiny);   // works for a4988 (Bipolar, constant current, step/direction driver)
AccelStepper myStepperx(AccelStepper::DRIVER, stepPinx, dirPinx);

void setup() {
	// OTA
	ota.setHostname("espdoolhof");  
	ota.setPassword("espdoolhof");
	ota.begin();

  Serial.begin(115200);
  //MQTT -
  setup_wifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  

  //pin setup
  pinMode(switchy,INPUT_PULLUP);
  pinMode(switchx,INPUT_PULLUP);
  pinMode(finish,INPUT_PULLUP);
  pinMode(poty,INPUT);
  pinMode(potx,INPUT);
  //motor setup
  //Speeds and accelerations are for when using x16 microstepping
  mySteppery.setMaxSpeed(16000.0);    // must be equal to or greater than desired speed.
  mySteppery.setSpeed(16000.0);       // desired speed to run at
  mySteppery.setAcceleration(9500); // desired acceleration
  myStepperx.setMaxSpeed(16000.0);    // must be equal to or greater than desired speed.
  myStepperx.setSpeed(16000.0);       // desired speed to run at
  myStepperx.setAcceleration(9500); // desired acceleration

  Serial.println("begin setup");

  //find home position
  while(!digitalRead(switchy)){
	if (!mySteppery.run()){
		mySteppery.move(-5);
	}
	taskYIELD(); //needed for OTA to work properly (otherwise OTA update will fail) since OTA is running on a different task
  }
  while(!digitalRead(switchx)){
	if (!myStepperx.run()){
		myStepperx.move(5);

	}
	taskYIELD();
  }
  Serial.println("home position found");
  //set home position
  mySteppery.setCurrentPosition(0);
  myStepperx.setCurrentPosition(0);
  Serial.println("setup done");
}



void loop() {
	//MQTT -
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();
    //- MQTT

	//mysteppery.stop() //niet nodig denk ikkkkkkkkkkkkkkk
  //nog proberen of mystepper.stop() beter werkt--------------------------------------------!!!!!!!!!!!!!!!!!!!!!---------------------------------

	//while(!digitalRead(finish)){
    //move x to the measured position of the potentiometer
		value_potx = analogRead(potx);
		posx = (value_potx*max_rotx)/4095;
    myStepperx.moveTo(posx);
    myStepperx.run();
		// move y to the measured position of the potentiometer
		value_poty = analogRead(poty);
		posy = -(value_poty*max_roty)/4095;
    mySteppery.moveTo(posy);
    mySteppery.run();
		taskYIELD();
  	//}
    //when the ball has reached the finish line, publish a message to the MQTT broker
    //client.publish(topic, "datum");
}