/*
Speeds: 1500 ... 2000 Forward
        1000 ... 500 Reverse
        1480 Brake

Steering: 90 ... 50 left
          90 ... 130 right

Sensor: 20 (far) ... 400 (close)

*/

// Variables

#define TARGET_DISTANCE 200
#define SPEED 1800
#define MOTOR_SETUP 0
#define QUIET_MODE 1

// Speed tuning (ESC in microseconds)
#define SPEED_NEUTRAL_US 1500
#define SPEED_BRAKE_US 1480
#define SPEED_MIN_US 1680
#define SPEED_MAX_US 1800

// Simple distance-hold PID using middle sensor (error = TARGET_DISTANCE - middle)
// Output is ESC PWM in microseconds, clamped to forward + brake (no reverse).
#define GESCHWINDIGKEITSREGELUNG 1
#define REGLER_V_P 0.8f
#define REGLER_V_I 0.02f
#define REGLER_V_D 0.3f
#define REGLER_V_DEADBAND 5

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

// Track offset: drive ~30% to the right instead of centered.
// Implemented as a bias on the left-right difference setpoint.
// 0.0f => center, 0.30f => right sensor can be ~30% "closer" (higher ADC) than left.
#define REGLER_LR_RIGHT_BIAS_FRAC 0.30f

// Simple segment recognition (left curve / right curve / straight)
#define SEG_JUMP_THRESHOLD 40
#define SEG_SIMILAR_THRESHOLD 50
#define SEG_NEAR_THRESHOLD 250
#define SEG_CONFIRM_COUNT 3

// WallRecovery: recovery when "in the wall"
#define WALL_HIT_THRESHOLD 400
#define WALL_HIT_CONFIRM_COUNT 3
// If left/right differ by this much, assume the wall is more on that side.
// (Higher ADC means closer.) Used only for the WallRecovery sequence.
#define WALLRECOVERY_SIDE_DIFF_THRESHOLD 40
// If WallRecovery triggers again within this window, reuse the previous
// reverse steering direction.
#define WALLRECOVERY_REPEAT_WINDOW_MS 5000UL
#define WALLRECOVERY_REVERSE_US 800
#define WALLRECOVERY_REVERSE_MS 400
#define WALLRECOVERY_FORWARD_US SPEED
#define WALLRECOVERY_FORWARD_MS 700

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

    // Bias the setpoint so we drive more to the right.
    // desiredDiff is negative when we want rightDistance > leftDistance.
    const float avgLR = 0.5f * (float)(leftDistance + rightDistance);
    const float desiredDiff = -REGLER_LR_RIGHT_BIAS_FRAC * avgLR;
    err_0 = (float)(leftDistance - rightDistance) - desiredDiff;

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

static int calcSpeedUsFromMiddlePID(int middleDistance, bool enabled)
{
    // Incremental (velocity-form) PID similar to the provided example.
    static float err_0 = 0.0f;
    static float err_1 = 0.0f;
    static float err_2 = 0.0f;
    static float pidOutUs = (float)SPEED;

    if (!enabled)
    {
        err_0 = 0.0f;
        err_1 = 0.0f;
        err_2 = 0.0f;
        pidOutUs = (float)SPEED;
        return SPEED_BRAKE_US;
    }

    err_2 = err_1;
    err_1 = err_0;
    err_0 = (float)(TARGET_DISTANCE - middleDistance); // >0 => too far => speed up

    if (fabsf(err_0) < (float)REGLER_V_DEADBAND)
        err_0 = 0.0f;

    pidOutUs = pidOutUs + (REGLER_V_P * (err_0 - err_1)) + (REGLER_V_I * (err_1)) + (REGLER_V_D * (err_0 - 2.0f * err_1 + err_2));

    if (pidOutUs > (float)SPEED_MAX_US)
        pidOutUs = (float)SPEED_MAX_US;
    if (pidOutUs < (float)SPEED_MIN_US)
        pidOutUs = (float)SPEED_MIN_US;

    return (int)lroundf(pidOutUs);
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

    (void)vBat;

    // map to real voltage (10k/47k divider and 5V ref)
    float vBatFloat = vBat * 0.0152867037f;
    if (QUIET_MODE == 0)
    {
        Serial.print("VBat: ");
        Serial.print(vBatFloat);
    }
    if (runMode == 0 && QUIET_MODE == 0)
        Serial.println(" (not running)");

    if (vBatFloat < 3.3f)
    {
        Serial.println("Battery too low!");
        runMode = 0;
        speedServo.writeMicroseconds(SPEED_BRAKE_US);
        steeringServo.write(STEERING_NEUTRAL_DEG);
    }
    // Now check the Button and run control strategies

    if (digitalRead(STOPBUTTON) == LOW)
    {
        runMode = 0;                                  // stopen des Fahrzeugs
        speedServo.writeMicroseconds(SPEED_BRAKE_US); // set into Brake mode
        steeringServo.write(STEERING_NEUTRAL_DEG);    // set steering to neutral
    }
    else
    {
        if (digitalRead(STARTBUTTON) == LOW)
            runMode = 1; // fahrzeug darf fahren
    }

    // WallRecovery (simple blocking sequence)
    static uint8_t wallHitCount = 0;
    static int lastSteeringDeg = STEERING_NEUTRAL_DEG;
    static unsigned long lastWallRecoveryMs = 0;
    static int lastWallRecoveryReverseSteerDeg = STEERING_NEUTRAL_DEG;

    // Kurvenmodus: wenn ein Seitensensor sprunghaft "weit weg" wird (ADC sinkt),
    // dann stark in die entsprechende Richtung lenken, bis derselbe Sensor wieder
    // "nah" an der Wand ist.
    static uint8_t curveMode = 0; // 0=aus, 1=links, 2=rechts
    static int prevLeftDistance = -1;
    static int prevRightDistance = -1;

    if (prevLeftDistance < 0)
        prevLeftDistance = leftDistance;
    if (prevRightDistance < 0)
        prevRightDistance = rightDistance;

    if (runMode == 0)
    {
        wallHitCount = 0;
        lastWallRecoveryMs = 0;
        curveMode = 0;
    }

    if (runMode == 1)
    {

        // --- Kurvenmodus Erkennung / Exit ---
        // ADC sinkt => Abstand wird größer. Ein starker Abfall bedeutet: Wand auf
        // dieser Seite "öffnet" sich -> Kurve in diese Richtung.
        if (curveMode == 0)
        {
            const int leftDrop = prevLeftDistance - leftDistance;
            const int rightDrop = prevRightDistance - rightDistance;

            if ((leftDrop >= SEG_JUMP_THRESHOLD) || (rightDrop >= SEG_JUMP_THRESHOLD))
            {
                // Wenn beides triggert, nimm den stärkeren Drop.
                curveMode = (leftDrop >= rightDrop) ? 1 : 2;
            }
        }
        else if (curveMode == 1)
        {
            if (leftDistance >= SEG_NEAR_THRESHOLD)
                curveMode = 0;
        }
        else if (curveMode == 2)
        {
            if (rightDistance >= SEG_NEAR_THRESHOLD)
                curveMode = 0;
        }

        // Geschwindigkeit
        const int speedUs = calcSpeedUsFromMiddlePID(middleDistance, GESCHWINDIGKEITSREGELUNG == 1);
        speedServo.writeMicroseconds(speedUs);

        int steeringDeg;

        steeringDeg = calcSteeringDegFromLeftRightPID(leftDistance, rightDistance, ANTRIEBSREGELUNG == 1);

        // Im Kurvenmodus: starker Einschlag (PID bleibt unverändert, wird nur übersteuert)
        if (curveMode == 1)
            steeringDeg = STEERING_MIN_DEG;
        else if (curveMode == 2)
            steeringDeg = STEERING_MAX_DEG;

        steeringDeg = constrain(steeringDeg, STEERING_MIN_DEG, STEERING_MAX_DEG);
        steeringServo.write(steeringDeg);

        lastSteeringDeg = steeringDeg;

        // Detect wall hit -> switch to Reverse Mode
        if (middleDistance >= WALL_HIT_THRESHOLD)
        {
            if (wallHitCount < 255)
                wallHitCount++;
        }
        else
        {
            wallHitCount = 0;
        }

        if (wallHitCount >= WALL_HIT_CONFIRM_COUNT)
        {
            wallHitCount = 0;

            const unsigned long nowMs = millis();

            // If we hit the wall at an angle, pick the recovery steering based on
            // which side is closer to the wall. (Higher ADC => closer.)
            // Note: When reversing, steering *towards* the wall side tends to swing
            // the front away from the wall.
            const int lrDiff = leftDistance - rightDistance;
            int steerReverseDeg;
            if (lrDiff >= WALLRECOVERY_SIDE_DIFF_THRESHOLD)
            {
                // Left is closer => steer left while reversing
                steerReverseDeg = STEERING_MIN_DEG;
            }
            else if (lrDiff <= -WALLRECOVERY_SIDE_DIFF_THRESHOLD)
            {
                // Right is closer => steer right while reversing
                steerReverseDeg = STEERING_MAX_DEG;
            }
            else
            {
                // Fallback: keep previous behavior based on last steering direction
                const bool wasTurningRight = (lastSteeringDeg >= STEERING_NEUTRAL_DEG);
                steerReverseDeg = wasTurningRight ? STEERING_MAX_DEG : STEERING_MIN_DEG;
            }

            // If we have to recover again shortly after, reuse the previous
            // reverse steering direction to keep the behavior consistent.
            if ((lastWallRecoveryMs != 0) && ((nowMs - lastWallRecoveryMs) <= WALLRECOVERY_REPEAT_WINDOW_MS))
            {
                steerReverseDeg = lastWallRecoveryReverseSteerDeg;
            }

            const int steerForwardDeg = (steerReverseDeg == STEERING_MAX_DEG) ? STEERING_MIN_DEG : STEERING_MAX_DEG;

            lastWallRecoveryMs = nowMs;
            lastWallRecoveryReverseSteerDeg = steerReverseDeg;

            // Reverse, full lock same direction
            steeringServo.write(steerReverseDeg);
            speedServo.writeMicroseconds(WALLRECOVERY_REVERSE_US);
            delay(WALLRECOVERY_REVERSE_MS);
            speedServo.writeMicroseconds(SPEED_BRAKE_US);
            delay(200);

            // Forward, full lock opposite direction
            steeringServo.write(steerForwardDeg);
            speedServo.writeMicroseconds(WALLRECOVERY_FORWARD_US);
            delay(WALLRECOVERY_FORWARD_MS);

            // Back to normal mode, reset PIDs so they don't "jump"
            speedServo.writeMicroseconds(SPEED_BRAKE_US);
            steeringServo.write(STEERING_NEUTRAL_DEG);
            (void)calcSteeringDegFromLeftRightPID(leftDistance, rightDistance, false);
            (void)calcSpeedUsFromMiddlePID(middleDistance, false);
        }

        if (QUIET_MODE == 0)
        {
            Serial.print("\tLeft: ");
            Serial.print(leftDistance);
            Serial.print("\tMiddle: ");
            Serial.print(middleDistance);
            Serial.print("\tRight: ");
            Serial.print(rightDistance);
            Serial.print("\tSpeedUs: ");
            Serial.print(speedUs);
            Serial.print("\tSteeringDeg: ");
            Serial.println(steeringDeg);
        }
    }
    else
    {
        // reset PID state when not driving, so next start begins centered
        (void)calcSteeringDegFromLeftRightPID(leftDistance, rightDistance, false);
        (void)calcSpeedUsFromMiddlePID(middleDistance, false);
    }

    // Update previous samples for next iteration
    prevLeftDistance = leftDistance;
    prevRightDistance = rightDistance;
}
