/*
#include <Arduino.h>

// Define pin connections & motor's steps per revolution
const int dirPin = 18;
const int stepPin = 19;
const int stepsPerRevolution = 3200;

void setup()
{
	// Declare pins as Outputs
    pinMode(2,INPUT);
	pinMode(stepPin, OUTPUT);
	pinMode(dirPin, OUTPUT);
	pinMode(4,INPUT_PULLUP);
    Serial.begin(9600);
}
void loop()
{
	while(digitalRead(4)){
	Serial.println(analogRead(2));
    delay(100);
	}
    Serial.println(digitalRead(4));
}*/



#include <AccelStepper.h>


// Motor Connections (constant current, step/direction bipolar motor driver)
const int dirPiny = 18;
const int stepPiny = 19;
const int dirPinx = 33;
const int stepPinx = 32;
const int switchy = 4;
const int switchx = 14;
const int poty = 2;
const int potx = 12;
int posy = 0;
int value_poty = 0;
const int max_rot = 255; 



AccelStepper mySteppery(AccelStepper::DRIVER, stepPiny, dirPiny);   // works for a4988 (Bipolar, constant current, step/direction driver)
AccelStepper myStepperx(AccelStepper::DRIVER, stepPinx, dirPinx);

void setup() {
  Serial.begin(9600);
  // set the maximum speed and initial speed. The initial speed will be the only
  // speed used. No acceleration will happen - only runSpeed is used. Runs forever.
  pinMode(switchy,INPUT_PULLUP);
  pinMode(switchx,INPUT_PULLUP);
  pinMode(poty,INPUT);
  pinMode(potx,INPUT);
  mySteppery.setMaxSpeed(1000.0);    // must be equal to or greater than desired speed.
  mySteppery.setSpeed(700.0);       // desired speed to run at
  mySteppery.setAcceleration(350);
  Serial.println("in setup");

  while(!digitalRead(switchy)){
	if (!mySteppery.run()){
		mySteppery.move(-5);
	}
	
  }
  mySteppery.setCurrentPosition(0);
  Serial.println("na de while");
  posy = 0;
}

void loop() {
	//mysteppery.stop()
	if (!mySteppery.run()){
		
	}
		value_poty = analogRead(poty);
		Serial.println(value_poty);
		
		mySteppery.moveTo(value_poty/30);
		delay(20);

	//if (!mySteppery.run()){
	//	mySteppery.moveTo(-mySteppery.currentPosition());
	//}
  	
  	
}


/*
#include <Arduino.h>

// Define pin connections & motor's steps per revolution
const int dirPin = 18;
const int stepPin = 19;
const int stepsPerRevolution = 3200;


void setup()
{
	// Declare pins as Outputs
	pinMode(stepPin, OUTPUT);
	pinMode(dirPin, OUTPUT);
}
void loop()
{
	// Set motor direction clockwise
	digitalWrite(dirPin, HIGH);

	// Spin motor slowly
	for(int x = 0; x < stepsPerRevolution; x++)
	{
		digitalWrite(stepPin, HIGH);
		delayMicroseconds(500);
		digitalWrite(stepPin, LOW);
		delayMicroseconds(500);
	}
	delay(1000); // Wait a second
	
	// Set motor direction counterclockwise
	digitalWrite(dirPin, LOW);

	// Spin motor quickly
	for(int x = 0; x < stepsPerRevolution; x++)
	{
		digitalWrite(stepPin, HIGH);
		delayMicroseconds(1000);
		digitalWrite(stepPin, LOW);
		delayMicroseconds(1000);
	}
	delay(1000); // Wait a second
} 
*/