/*
Speeds: 1500 ... 2000 Forward
        1000 ... 500 Reverse
        1480 Brake

Steering: 90 ... 50 left
          90 ... 130 right

Sensor: 20 (far) ... 500 (close)

*/

// Variables

#define TARGET_DISTANCE 200
#define SPEED 1800
#define MOTOR_SETUP 0

// Steering tuning
#define STEERING_NEUTRAL_DEG 90
#define STEERING_MIN_DEG 50
#define STEERING_MAX_DEG 130

// Simple left/right centering PID (error = left - right; >0 => steer right)
// Output is kept in 0..100 and mapped to steering angle.
#define ANTRIEBSREGELUNG 1
#define REGLER_LR_P 0.35f
#define REGLER_LR_I 0.01f
#define REGLER_LR_D 0.15f
#define REGLER_LR_DEADBAND 5

// Simple obstacle handling: if middle sensor sees close obstacle, force right turn
#define MIDDLE_OBSTACLE_THRESHOLD 200
#define MIDDLE_OBSTACLE_STEER_DEG STEERING_MAX_DEG

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

static int calcSteeringDegFromLeftRightPID(int leftDistance, int rightDistance, bool enabled)
{
    // Incremental (velocity-form) PID as in the user's example.
    static float err_0 = 0.0f;
    static float err_1 = 0.0f;
    static float err_2 = 0.0f;
    static float pidOut = 50.0f; // 0..100, 50 = center

    if (!enabled)
    {
        err_0 = 0.0f;
        err_1 = 0.0f;
        err_2 = 0.0f;
        pidOut = 50.0f;
        return STEERING_NEUTRAL_DEG;
    }

    err_2 = err_1;
    err_1 = err_0;
    err_0 = (float)(leftDistance - rightDistance); // setpoint is 0 => centered

    if (fabsf(err_0) < (float)REGLER_LR_DEADBAND)
        err_0 = 0.0f;

    pidOut = pidOut + (REGLER_LR_P * (err_0 - err_1)) + (REGLER_LR_I * (err_1)) + (REGLER_LR_D * (err_0 - 2.0f * err_1 + err_2));

    if (pidOut > 100.0f)
        pidOut = 100.0f;
    if (pidOut < 0.0f)
        pidOut = 0.0f;

    // Map 0..100 to steering min..max
    const float steeringSpan = (float)(STEERING_MAX_DEG - STEERING_MIN_DEG);
    const float steeringDegF = (float)STEERING_MIN_DEG + (pidOut / 100.0f) * steeringSpan;
    return constrain((int)lroundf(steeringDegF), STEERING_MIN_DEG, STEERING_MAX_DEG);
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

    (void)rightDistance;
    (void)vBat;

    Serial.print("VBat: ");
    float vBatFloat = vBat * 0.010089f; // map to real voltage (10k/47k divider and 5V ref)
    Serial.print(vBatFloat);
    if (runMode == 0)
        Serial.println(" (not running)");

    if (vBatFloat < 3.3f)
    {
        Serial.println("Battery too low!");
        runMode = 0;
    }
    // Now check the Button and run control strategies

    if (digitalRead(STOPBUTTON) == LOW)
    {
        runMode = 0;                        // stopen des Fahrzeugs
        speedServo.writeMicroseconds(1480); // set into Brake mode
        steeringServo.write(90);            // set steering to neutral
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

        int steeringDeg;
        if (middleDistance >= MIDDLE_OBSTACLE_THRESHOLD)
        {
            steeringDeg = MIDDLE_OBSTACLE_STEER_DEG;
        }
        else
        {
            steeringDeg = calcSteeringDegFromLeftRightPID(leftDistance, rightDistance, ANTRIEBSREGELUNG == 1);
        }

        steeringDeg = constrain(steeringDeg, STEERING_MIN_DEG, STEERING_MAX_DEG);
        steeringServo.write(steeringDeg);

        Serial.print("\tLeft: ");
        Serial.print(leftDistance);
        Serial.print("\tMiddle: ");
        Serial.print(middleDistance);
        Serial.print("\tRight: ");
        Serial.print(rightDistance);
        Serial.print("\tSteeringDeg: ");
        Serial.println(steeringDeg);
    }
    else
    {
        // reset PID state when not driving, so next start begins centered
        (void)calcSteeringDegFromLeftRightPID(leftDistance, rightDistance, false);
    }
}
