/*
Speeds: 1500 ... 2000 Forward
        1000 ... 500 Reverse
        1480 Brake

Steering: 90 ... 50 left
          90 ... 140 right

Sensor: 20 (far) ... 500 (close)

*/

// Variables

#define TARGET_DISTANCE 200
#define SPEED 1700
#define MOTOR_SETUP 0

// Steering tuning (simple proportional control based on LEFT sensor only)
#define STEERING_NEUTRAL_DEG 90
#define STEERING_MIN_DEG 50
#define STEERING_MAX_DEG 140
#define STEERING_DEADBAND 10
#define STEERING_KP_DIV 6

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

static int calcSteeringDegFromLeft(int leftDistance)
{
    const int error = leftDistance - TARGET_DISTANCE;
    if (abs(error) <= STEERING_DEADBAND)
        return STEERING_NEUTRAL_DEG;

    int offset = error / STEERING_KP_DIV; // closer to wall (bigger value) => steer more to the right
    int steeringDeg = STEERING_NEUTRAL_DEG + offset;
    return constrain(steeringDeg, STEERING_MIN_DEG, STEERING_MAX_DEG);
}

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
    if (MOTOR_SETUP == 1)
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
    {
        runMode = 0;                        // stopen des Fahrzeugs
        speedServo.writeMicroseconds(1480); // set into Brake mode
    }
    else
    {
        if (digitalRead(STARTBUTTON) == LOW)
            runMode = 1; // fahrzeug darf fahren
    }

    if (runMode == 1)
    {
        // Geschwindigkeit
        speedServo.writeMicroseconds(SPEED);

        const int steeringDeg = calcSteeringDegFromLeft(leftDistance);
        steeringServo.write(steeringDeg);

        Serial.print("Left: ");
        Serial.print(leftDistance);
        Serial.print(" SteeringDeg: ");
        Serial.println(steeringDeg);
    }
}
