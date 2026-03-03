#include <arduino.h>

/**********************************************************************
 DISPLAY
 **********************************************************************
 PIN Konfig
 52 SCK
 50 MISO
 51 MOSI /DIN
 53 SS /CS
 32 DC /A0
 31 RESET
 30 BackLight
 ***********************************************************************/


/***********************************************************************
 *SERVOs steering, speed
 ***********************************************************************
 * auf folgenden Arduino Pins befinden sich die Servoanschl�sse:
 *
 *  5 ... throttle / Steering
 *  6 ... esc / speed
 *  ********************************************************************/

#include <Servo.h>
#define PIN_STEERING_SERVO 5
#define PIN_SPEED_SERVO 2



Servo steeringServo;  // SteeringServo
Servo speedServo;  // speedServo

/*************************************************************************
 * IR-Length Sensors
 *************************************************************************
 * auf folgenden PINs befinden sich die IR Sensoren
 *
 * A0 ... VBATT_MEASUREMENT
 * A1 ... Distance Left
 * A2 ... Distance Right
 * A3 ... Distance Middle
 *
 * Anmerkung: Es kann die interne Referenzspannung verwendet werden
 *************************************************************************/

#define LEFTSENSOR A1
#define MIDDLESENSOR A3
#define RIGTHSENSOR A2
#define VBAT A0

/*************************************************************************/

/*************************************************************************
 * Start Stop Tasten
 *************************************************************************
 * auf folgenden PINs befinden sich die Start und die Stop Taste
 *
 * 13 ... Stop
 * 12 ... Start
 *
 * Anmerkung: die Tasten verwenden negative logik
 *************************************************************************/
#define STARTBUTTON 12 //schwarze Taste
#define STOPBUTTON 13  //rote Taste
unsigned long Zeit_us_1;
unsigned long Zeit_us_2;

/**************************************************************************
 * Global Variablen
 **************************************************************************/
unsigned char runMode = 0; //Variable = 0 ... stop

//The setup function is called once at startup of the sketch
void setup() {
	Serial.begin(9600);
	//attach Servo objects to the corresponding PINs
	startZeit_us = micros();

}

// The loop function is called in an endless loop
void loop() {

	//First get all data
	int leftDistance = analogRead(LEFTSENSOR);
	int middleDistance = analogRead(MIDDLESENSOR);
	int rightDistance = analogRead(RIGTHSENSOR);
	float distance;
	int vBat = analogRead(VBAT);
//	Serial.println(vBat*0.010089);




	Serial.print("LD: ");	
	Serial.print(leftDistance);
	distance = 0.0003*leftDistance*leftDistance - 0.2269*leftDistance + 57.126;
	Serial.print("  Dist.: ");	
	Serial.println(distance);
	delay(500);




}



void setupESCPWM() {
    const int CONFIG_DELAY = 5000;//Zeit wie lange das Konfig Signal anliegt
	speedServo.writeMicroseconds(500);//setup maximum Reverse
	delay(CONFIG_DELAY);
	speedServo.writeMicroseconds(1000);//setup neutral zone Reverse|Brake
	delay(CONFIG_DELAY);
	speedServo.writeMicroseconds(1500);//setup Neutral zone Brake Forward
	delay(CONFIG_DELAY);
	speedServo.writeMicroseconds(2500);//setup Forward Max
	delay(CONFIG_DELAY);
	speedServo.writeMicroseconds(1480);//set into Brake mode
	delay(CONFIG_DELAY);
}
