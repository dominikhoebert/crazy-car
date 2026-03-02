// Variables

#define TARGET_DISTANCE 200
#define SPEED 1600

#include <Arduino.h>
/***********************************************************************
 *SERVOs steering, speed
 ***********************************************************************
 * auf folgenden Arduino Pins befinden sich die Servoanschlüsse:
 *
 *  5 ... throttle / Steering
 *  6 ... esc / speed
 *  ********************************************************************/

#include <Servo.h>
#define PIN_STEERING_SERVO 5
#define PIN_SPEED_SERVO 2

Servo steeringServo; // SteeringServo
Servo speedServo;    // speedServo

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
#define STARTBUTTON 12 // schwarze Taste
#define STOPBUTTON 13  // rote Taste

/**************************************************************************
 * Global Variablen
 **************************************************************************/
unsigned char runMode = 0; // Variable = 0 ... stop

void setupESCPWM()
{
    const int CONFIG_DELAY = 5000;     // Zeit wie lange das Konfig Signal anliegt
    speedServo.writeMicroseconds(500); // setup maximum Reverse
    delay(CONFIG_DELAY);
    speedServo.writeMicroseconds(1000); // setup neutral zone Reverse|Brake
    delay(CONFIG_DELAY);
    speedServo.writeMicroseconds(1500); // setup Neutral zone Brake Forward
    delay(CONFIG_DELAY);
    speedServo.writeMicroseconds(2500); // setup Forward Max
    delay(CONFIG_DELAY);
    speedServo.writeMicroseconds(1480); // set into Brake mode
    delay(CONFIG_DELAY);
}

// The setup function is called once at startup of the sketch
void setup()
{
    // attach Servo objects to the corresponding PINs
    steeringServo.attach(PIN_STEERING_SERVO);
    steeringServo.write(90); // Set it to neutral position
    speedServo.attach(PIN_SPEED_SERVO);
    setupESCPWM();
    Serial.begin(9600);
    Serial.println("Setup done");
}

// The loop function is called in an endless loop
void loop()
{
    // First get all data
    int leftDistance = analogRead(LEFTSENSOR);
    int middleDistance = analogRead(MIDDLESENSOR);
    int rightDistance = analogRead(RIGTHSENSOR);
    int vBat = analogRead(VBAT);

    // Now check the Button and run control strategies

    if (digitalRead(STOPBUTTON) == LOW)
        runMode = 0; // stopen des Fahrzeugs
    else
    {
        if (digitalRead(STARTBUTTON) == LOW)
            runMode = 1; // fahrzeug darf fahren
    }

    if (runMode == 1)
    {
        // Geschwindigkeit
        speedServo.writeMicroseconds(SPEED); // lowspeed ahead

        // Lenkung (Steering)
        int lenkung = ((long)(leftDistance - middleDistance)) * 90 / 1024;
        Serial.print("left: ");
        Serial.print(leftDistance);
        Serial.print(" middle: ");
        Serial.print(middleDistance);
        Serial.print(" lenkung: ");
        Serial.println(lenkung);

        steeringServo.write(lenkung + 90);
    }
}
