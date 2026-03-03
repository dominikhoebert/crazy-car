#include "U8glib.h"
#include <arduino.h>



//U8GLIB_PCD8544 u8g(13, 11, 10,9, 8);// SCK, MOSI, CS, A0,RESET .... SW SPI
#define BACKLIGHT 7
#define FONTHEIGTH 7

/***********************************************************************
 *SERVOs steering, speed
 ***********************************************************************
 * auf folgenden Arduino Pins befinden sich die Servoanschl�sse:
 *
 *  5 ... Platinenbeschriftung TH
 *  6 ... Platinenbeschriftung ST
 *  ********************************************************************/

#include <Servo.h>

Servo steeringServo;  // SteeringServo
Servo speedServo;  // SteeringServo

/*************************************************************************
 * IR-Length Sensors
 *************************************************************************
 * auf folgenden PINs befinden sich die IR Sensoren
 *
 * A0 ... Platinenbeschriftung CO12
 * A1 ... Platinenbeschriftung CO13
 * A2 ... Platinenbeschriftung CO14
 * A3 ... VBat
 *
 * Anmerkung: am AREF Eingang des ADCs liegt eine Referenzspannung von 2.048V
 *************************************************************************/

#define LEFTSENSOR A0
#define MIDDLESENSOR A1
#define RIGTHSENSOR A2
#define VBAT A3

/*************************************************************************/

/*************************************************************************
 * Start Stop Tasten
 *************************************************************************
 * auf folgenden PINs befinden sich die Start und die Stop Taste
 *
 * 3 ... Stop
 * 4 ... Start
 *
 * Anmerkung: die Tasten verwenden negative logik
 *************************************************************************/
#define STARTBUTTON 12 //Platinenbeschriftung Start
#define STOPBUTTON 13  //Platinenbeschriftung Stop

/**************************************************************************
 * Global Variablen
 **************************************************************************/
unsigned char runMode = 0; //Variable = 0 ... stop

//The setup function is called once at startup of the sketch
void setup() {
	//attach Servo objects to the corresponding PINs
	steeringServo.attach(5);
	steeringServo.write(90); //Set it to neutral position
	speedServo.attach(6);
	speedServo.write(90); //Set it to neutral position

	Serial.begin(9600);
	Serial.write("Funktioniert!");

	pinMode(STOPBUTTON, INPUT);

}

// The loop function is called in an endless loop
void loop() {

	//First get all data
	int leftDistance = analogRead(LEFTSENSOR);
	int middleDistance = analogRead(MIDDLESENSOR);
	int rightDistance = analogRead(RIGTHSENSOR);
	int vBat = analogRead(VBAT);


	//Now check the Button and run control strategies
		if (digitalRead(STOPBUTTON) == LOW)
		{
			runMode = 1; //fahrzeug darf fahren
			Serial.println("1");
		}
		else
			Serial.println("0");


	if (runMode == 1) {
		//Geschwindigkeit
		speedServo.write(110); //lowspeed ahead

		//Lenkung (Steering)
		int lenkung = ((long) (leftDistance - middleDistance)) * 90 / 1024;
		steeringServo.write(lenkung + 90);
	}


}

