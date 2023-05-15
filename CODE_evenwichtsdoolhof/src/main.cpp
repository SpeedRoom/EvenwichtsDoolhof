
//Autor: Emiel Dever, Jonathan Gheysens

#include <AccelStepper.h>
#include <WiFi.h>
#include "OTAlib.h"
#include <PubSubClient.h>


//WiFi settings and MQTT settings
//change to your own settings
#define SSID          "NETGEAR68"
#define PWD           "excitedtuba713"
#define MQTT_SERVER   "192.168.1.61"  
#define MQTT_PORT     1883
#define topic  "esp_doolhof/output"
#define hostname "espdoolhof"
#define password "espdoolhof"

//Pin connections
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
//Stepper motor settings
#define maxspeed 20000
#define accel 28000
#define speed 20000

int posy = 0;
int posx = 0;
int value_poty = 0;
int value_potx = 0;
bool finished = false;
const int max_roty = 2850;
const int max_rotx = 2300;

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

WiFiClient espClient;
PubSubClient client(espClient);
OTAlib ota(SSID, PWD);
AccelStepper mySteppery(AccelStepper::DRIVER, stepPiny, dirPiny);   // works for a4988 (Bipolar, constant current, step/direction driver)
AccelStepper myStepperx(AccelStepper::DRIVER, stepPinx, dirPinx);

void setup() {
  Serial.begin(115200);
  
	// OTA
	ota.setHostname(hostname);  
	ota.setPassword(password);
	ota.begin();

  //MQTT -
  client.setServer(MQTT_SERVER, MQTT_PORT);
  
  //pin setup
  pinMode(switchy,INPUT_PULLUP);
  pinMode(switchx,INPUT_PULLUP);
  pinMode(finish,INPUT_PULLUP);
  pinMode(poty,INPUT);
  pinMode(potx,INPUT);
  
  //motor setup
  //Speeds and accelerations are for when using x16 microstepping
  mySteppery.setMaxSpeed(maxspeed);    // must be equal to or greater than desired speed.
  mySteppery.setSpeed(speed);       // desired speed to run at
  mySteppery.setAcceleration(accel); // desired acceleration
  myStepperx.setMaxSpeed(maxspeed);    // must be equal to or greater than desired speed.
  myStepperx.setSpeed(speed);       // desired speed to run at
  myStepperx.setAcceleration(accel); // desired acceleration
  
  //find home position
  while(!digitalRead(switchy)){
	if (!mySteppery.run()){
		mySteppery.move(-50);

	}
  //needed for OTA to work properly (otherwise OTA update will fail) since OTA is running on a different task
	taskYIELD(); 
  }
  mySteppery.setCurrentPosition(0);
  while(!digitalRead(switchx)){
	if (!myStepperx.run()){
		myStepperx.move(50);

	}
	taskYIELD();
  }
  myStepperx.setCurrentPosition(0);
  
}

void loop() {
  //MQTT -
  if (!client.connected()){
    reconnect();
  }
  client.loop();
  //- MQTT

	while(!finished){
    //move x to the measured position of the potentiometer
		value_potx = analogRead(potx);
		posx = (value_potx*max_rotx)/4095;
    myStepperx.moveTo(posx);
    myStepperx.run();

		// move y to the measured position of the potentiometer
		value_poty = analogRead(poty);
		posy = (value_poty*max_roty)/4095;
    mySteppery.moveTo(posy);
    mySteppery.run();
		taskYIELD();
    finished = digitalRead(finish);
  }
  //when the ball has reached the finish line, publish a message to the MQTT broker
  client.publish(topic, "datum");
}