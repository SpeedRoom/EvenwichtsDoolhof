
// naam esp: espdoolhof
// wachtwoord esp: espdoolhof

#include <AccelStepper.h>
#include <WiFi.h>
#include "OTAlib.h"
#include <PubSubClient.h>

//OTA
OTAlib ota("NETGEAR68", "excitedtuba713");

//MQTT -
#define SSID          "NETGEAR68"
#define PWD           "excitedtuba713"
#define MQTT_SERVER   "192.168.1.61"  
#define MQTT_PORT     1883
#define topic  "esp_doolhof/output"

WiFiClient espClient;
PubSubClient client(espClient);

const char *ssid = "NETGEAR68";
const char *password = "excitedtuba713";

// Pin connections
const int finish = 12;
const int dirPiny = 18;
const int stepPiny = 19;
const int dirPinx = 33;
const int stepPinx = 32;
const int switchy = 14;
const int switchx = 27;
//const int poty = 26;
//const int potx = 25;
const int poty = 39;
const int potx = 35;
int posy = 0;
int value_poty = 0;
int posx = 0;
int value_potx = 0;
const int max_roty = 1920;
const int max_rotx = 2720;

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

//MQTT -
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
//- MQTT




AccelStepper mySteppery(AccelStepper::DRIVER, stepPiny, dirPiny);   // works for a4988 (Bipolar, constant current, step/direction driver)
AccelStepper myStepperx(AccelStepper::DRIVER, stepPinx, dirPinx);

void setup() {
	// OTA
	ota.setHostname("espdoolhof");  
	ota.setPassword("espdoolhof");
	ota.begin();

  //MQTT -
  Serial.begin(115200);
  setup_wifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);

  vTaskDelay(100);
  

  pinMode(2,OUTPUT);
  //pin setup
  pinMode(switchy,INPUT_PULLUP);
  pinMode(switchx,INPUT_PULLUP);
  pinMode(finish,INPUT_PULLUP);
  pinMode(poty,INPUT);
  pinMode(potx,INPUT);
  pinMode(2,OUTPUT);
  //motor setup
  mySteppery.setMaxSpeed(16000.0);    // must be equal to or greater than desired speed.
  mySteppery.setSpeed(16000.0);       // desired speed to run at
  mySteppery.setAcceleration(9500); // desired acceleration
  myStepperx.setMaxSpeed(16000.0);    // must be equal to or greater than desired speed.
  myStepperx.setSpeed(16000.0);       // desired speed to run at
  myStepperx.setAcceleration(9500); // desired acceleration
  //standard value of speed 14400 and acceleration 8800
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
  posy = 0;
  posx = 0;
  Serial.println("setup done");
   if (!client.connected())
    {
        reconnect();
    }
    client.loop();
  //flashing led for visual feedback
  digitalWrite(2, HIGH);
  vTaskDelay(500);
  digitalWrite(2, LOW);
  vTaskDelay(500);
}



void loop() {
	//MQTT -
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

     //client.publish(topic, "datum"); // deze nog plaatsen waar gedetecteerd wordt dat balletje in het juiste gat is gevallen (en "datum" veranderen)
    //- MQTT

	//mysteppery.stop() //niet nodig denk ikkkkkkkkkkkkkkk
	//while(!digitalRead(finish)){
    //move x to the measured position of the potentiometer
    //digitalWrite(2, HIGH);
		value_potx = analogRead(potx);
		posx = (value_potx*max_rotx)/4095;
    //if(!myStepperx.run()){
      myStepperx.moveTo(posx);
    //}
    myStepperx.run();
		// move y to the measured position of the potentiometer
		value_poty = analogRead(poty);
		posy = -(value_poty*max_roty)/4095;
    mySteppery.moveTo(posy);
    mySteppery.run();


    //visuel feedback of end of loop function
		
		//delay(50);
		//digitalWrite(2, LOW);
		//delay(50);
		taskYIELD();
	
  	//}
    //mogelijks eerst connecten en dan pas publishen om te vermijden dat er iets verloren gaat
    //client.publish(topic, "datum");
}