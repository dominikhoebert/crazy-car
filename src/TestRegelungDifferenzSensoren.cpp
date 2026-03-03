#include <arduino.h>
#include <Servo.h>
#define PIN_STEERING_SERVO 5
#define PIN_SPEED_SERVO 2

#define STATE_INIT 0
#define STATE_NICHT_GESTARTET 5
#define STATE_GESTARTET 10
#define STATE_STOPPED 13
#define STATE_BLITZ_START 15
#define STATE_REGELUNG 20
#define STATE_LEFT_CURVE 30
#define STATE_RIGHT_CURVE 40

/*
Speeds: 1500 ... 2000 Forward
        1000 ... 500 Reverse
        1480 Brake

Steering: 90 ... 50 left
          90 ... 130 right

Sensor: 20 (far) ... 500 (close)

*/

#define TARGET_DISTANCE 200
#define SPEED 1900
#define SPEED_STOPPED 1480
#define SPEED_FULL 2200
#define SPEED_CURVE 1800
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

#define PIN_STEERING_SERVO 5
#define PIN_SPEED_SERVO 2

#define LEFTSENSOR A1
#define MIDDLESENSOR A3
#define RIGHTSENSOR A2
#define VBAT A0

#define DIFFERENCE_DETECTION_CURVE 30
#define DISTANCE_DETECTION_END_OF_CURVE 200

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

static int calcSteeringDegFromLeftRightPID(int leftDistance, int rightDistance, bool enabled);
void setupESCPWM();

Servo steeringServo;  // SteeringServo
Servo speedServo;  // speedServo

int main()
{
	unsigned char state = STATE_INIT; //Variable = 0 ... stop

	int leftDistance, middleDistance, rightDistance;
	int old_leftDistance, old_rightDistance;
	int startTime;
	
	init();

	Serial.begin(9600);

	while(1)
	{
		if (digitalRead(STOPBUTTON) == LOW)
			state = STATE_STOPPED;

		Serial.println(state);
		
		switch(state)
		{
		case STATE_INIT:
						// attach Servo objects to the corresponding PINs
			steeringServo.attach(PIN_STEERING_SERVO);
			steeringServo.write(90); // Set it to neutral position
			speedServo.attach(PIN_SPEED_SERVO);
			calcSteeringDegFromLeftRightPID(leftDistance, rightDistance, false);
			if (MOTOR_SETUP == 1)
				setupESCPWM();			
			
			state = STATE_NICHT_GESTARTET;
			break;
		case STATE_NICHT_GESTARTET:
			if (digitalRead(STARTBUTTON) == LOW)
			{
				startTime = millis();
				state = STATE_GESTARTET;
			}
			
			break;
		case STATE_GESTARTET:
			speedServo.writeMicroseconds(SPEED_FULL);

			if(millis() - startTime > 1000)
			{
				speedServo.writeMicroseconds(SPEED);
				state = STATE_REGELUNG;
			}

			break;

		case STATE_REGELUNG:
			old_leftDistance = leftDistance;
			old_rightDistance = rightDistance;
			middleDistance = analogRead(MIDDLESENSOR);
			rightDistance = analogRead(RIGHTSENSOR);
			leftDistance = analogRead(LEFTSENSOR);

			if(leftDistance - old_leftDistance > DIFFERENCE_DETECTION_CURVE)
			{
				state = STATE_LEFT_CURVE;
			}
			else if(rightDistance - old_rightDistance > DIFFERENCE_DETECTION_CURVE)
			{
				state = STATE_RIGHT_CURVE;
			}
			else
			{

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
			}
			/*Serial.print("Left: ");
			Serial.print(leftDistance);
			Serial.print(" Middle: ");
			Serial.print(middleDistance);
			Serial.print(" Right: ");
			Serial.print(rightDistance);
			Serial.print(" SteeringDeg: ");
			Serial.println(steeringDeg);
			*/
			break;

		case STATE_LEFT_CURVE:			
			speedServo.writeMicroseconds(SPEED_CURVE); // set into Brake mode
			steeringServo.write(STEERING_MIN_DEG); 
			leftDistance = analogRead(LEFTSENSOR);
			if(leftDistance > DISTANCE_DETECTION_END_OF_CURVE)
				state = STATE_REGELUNG;
		break;

		case STATE_RIGHT_CURVE:			
			speedServo.writeMicroseconds(SPEED_CURVE); // set into Brake mode
			steeringServo.write(STEERING_MIN_DEG); 
			rightDistance = analogRead(RIGHTSENSOR);
			if(rightDistance > DISTANCE_DETECTION_END_OF_CURVE)
				state = STATE_REGELUNG;
		break;

		case STATE_STOPPED:
			// stopen des Fahrzeugs
			speedServo.writeMicroseconds(SPEED_STOPPED); // set into Brake mode
			steeringServo.write(STEERING_NEUTRAL_DEG);            // set steering to neutral
			calcSteeringDegFromLeftRightPID(leftDistance, rightDistance, false);

			state = STATE_NICHT_GESTARTET;
			break;
		}



	}

	return 0;

}

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



