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

#define STATE_INIT 0
#define STATE_GESTARTET 10


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

/**************************************************************************
 * Global Variablen
 **************************************************************************/
unsigned char runMode = 0; //Variable = 0 ... stop

#define KP 10
#define KI 0.01
#define KD 0

#define TARGET_VALUE 0


int main()
{
	int e0, e1, e2;
	int leftDistance, middleDistance, rightDistance;
	int pidValue;
	
	Serial.begin(9600);
	//attach Servo objects to the corresponding PINs
	steeringServo.attach(PIN_STEERING_SERVO);
	steeringServo.write(90); //Set it to neutral position
	speedServo.attach(PIN_SPEED_SERVO);
	//setupESCPWM();
	speedServo.writeMicroseconds(1800); //lowspeed ahead

	e0=e1=e2=0;


	while(1)
	{
			//First get all data
		leftDistance = analogRead(LEFTSENSOR);
		middleDistance = analogRead(MIDDLESENSOR);
		rightDistance = analogRead(RIGTHSENSOR);

		e2 = e1;
		e1 = e0;
		e0 = (leftDistance - rightDistance) - TARGET_VALUE;

		pidValue = pidValue + KP*(e0-e1) + KI*(e1) + KD*(e0-2*e1+e2);
		steeringServo.write(60);
		
	}

	return 0;

}

