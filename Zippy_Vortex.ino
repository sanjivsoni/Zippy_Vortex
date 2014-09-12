
#include <Motor.h>
#include "QTRSensors_teensy3.h"

// PID Constants
#define KP	2
#define KI	0
#define KD	80

// MAX SPEED
#define SPEED_MAX 2048
#define SPEED_CALIBRATE 786
#define SPEED_TURN 1024

// Enable for white line on black background
#define WHITE_ON_BLACK 1

// QTR 8RC SETUP
#define NUMBER_OF_SENSORS 8
#define EMITTER_ON 1
#define EMITTER_PIN 22
#define TIMEOUT 2500

#define BUZZER_PIN 23

// Motor Driver Configurations
#define MOTOR_DRIVER_PIN_STANDBY 7
#define ENABLE_STANDBY digitalWrite(MOTOR_DRIVER_PIN_STANDBY, HIGH)
#define DISABLE_STANDBY digitalWrite(MOTOR_DRIVER_PIN_STANDBY, LOW)

// Buzzer Settings
#define BUZZER_ON digitalWrite(BUZZER_PIN, HIGH);
#define BUZZER_OFF digitalWrite(BUZZER_PIN, LOW);

#define DEBUG_MODE 1

motor motorLeft, motorRight;

// Sensor configs
unsigned char sensorPins[] = { 14, 15, 16, 17, 18, 19, 20, 21 };
unsigned short sensorValues[NUMBER_OF_SENSORS];

unsigned char ROBOT_STATE = 0;

#define BLUETOOTH Serial1

// PID Variables
float position_ = 0, proportional = 0, derivative = 0, integral = 0, lastProportional = 0;
float control = 0;

// Record the path
char path[100];
unsigned int pathCounter = 0;

// Initialize QTR8RC Sensor Array with sensor pins array
// and number of sensors as second parameter
QTRSensorsRC qtrRC(sensorPins, NUMBER_OF_SENSORS, TIMEOUT, EMITTER_PIN);

void turn(char direction,unsigned int speed,unsigned short delayTime)
{
	switch (direction)
	{
	case 'L':
		motorLeft.write(-speed);
		motorRight.write(speed);
		delay(delayTime);
		break;

	case 'R':
		motorLeft.write(speed);
		motorRight.write(-speed);
		delay(delayTime);
		break;

	case 'S':
		motorLeft.stop();
		motorRight.stop();
		DISABLE_STANDBY;

		break;
	}

}

void initializeBot()
{
	// Calibrate sensors 
	for (unsigned int i = 0; i < 90; i++)
	{
		if (i == 0)
			turn('L',SPEED_CALIBRATE,0);
		if (i == 30)
			turn('R', SPEED_CALIBRATE, 0);
		if (i == 70)
			turn('L', SPEED_CALIBRATE, 0);

		// Emitters on
		//if (WHITE_ON_BLACK == 1 && EMITTER_ON == 1)
		//	qtrRC.calibrate(QTR_EMITTERS_ON);

		// Emitters off
		//else
		qtrRC.calibrate();

	}

	//Display values serially in debugMode for Debugging
	if (DEBUG_MODE == 1)
	{
		for (unsigned char i = 0; i < NUMBER_OF_SENSORS; i++)
		{
			Serial.print(qtrRC.calibratedMinimumOn[i]);
			Serial.print(' ');
		}
		Serial.println(' ');
		for (unsigned char i = 0; i < NUMBER_OF_SENSORS; i++)
		{
			Serial.print(qtrRC.calibratedMinimumOff[i]);
			Serial.print(' ');
		}
	}
}

void showSensorValues()
{
	for (unsigned char i = 0; i < NUMBER_OF_SENSORS; i++)
	{
		Serial.print(sensorValues[i]);
		Serial.print(' ');
	}

	Serial.println();

}

void setup()
{
	if (DEBUG_MODE == 1)
	{
		Serial.begin(9600);
	}

	//BLUETOOTH.begin(9600);

	pinMode(MOTOR_DRIVER_PIN_STANDBY, OUTPUT);
	pinMode(13, OUTPUT);

	//DISABLE_STANDBY;
	ENABLE_STANDBY;

	pinMode(BUZZER_PIN, OUTPUT);

	motorLeft.setPins(6, 5, 4);
	motorRight.setPins(8, 9, 10);

	motorLeft.setMaxSpeed(SPEED_MAX);
	motorRight.setMaxSpeed(SPEED_MAX);

	motorLeft.initialise();
	motorRight.initialise();

	//12 bit width of analog write values
	analogWriteResolution(12);

	BLUETOOTH.begin(9600);

	digitalWrite(13, HIGH);
	BUZZER_ON;
	initializeBot();
	BUZZER_OFF;
	digitalWrite(13, LOW);

}

void runMappingMode()
{

	if (sensorValues[1] < 200 && sensorValues[2] < 200 && sensorValues[3] < 200 && sensorValues[4] < 200 && sensorValues[5] < 200 && sensorValues[6] < 200)
	{
		while (sensorValues[1] < 200 && sensorValues[2] < 200 && sensorValues[3] < 200 && sensorValues[4] < 200 && sensorValues[5] < 200 && sensorValues[6] < 200)
		{
			BUZZER_ON;
			position_ = qtrRC.readLine(sensorValues, QTR_EMITTERS_ON, true);
		}

		if (sensorValues[1] > 200 && sensorValues[2] > 200 && sensorValues[3] > 200 && sensorValues[4] > 200 && sensorValues[5] > 200 && sensorValues[6] > 200)
		{
			//Serial.print("90 Detected");
			turn('L', SPEED_TURN, 100);
			path[pathCounter++] = 'J';
			BLUETOOTH.print(path[pathCounter - 1]);
		}
		else
		{
			turn('S', 0, 0);
			path[pathCounter++] = 'S';
			path[pathCounter] = '\0';
			BLUETOOTH.println(path);

		}	
		BUZZER_OFF;
	}

	// Left 90 Turn
	else if (sensorValues[0] < 200 && sensorValues[1] < 200 && sensorValues[2] < 200 && sensorValues[3] < 200 && sensorValues[4] < 200)
	{

		while (sensorValues[0] < 200 && sensorValues[1] < 200 && sensorValues[2] < 200 && sensorValues[3] < 200 && sensorValues[4] < 200)
		{
			BUZZER_ON;
			position_ = qtrRC.readLine(sensorValues, QTR_EMITTERS_ON, true);
		}
		if (sensorValues[1] > 200 && sensorValues[2] > 200 && sensorValues[3] > 200 && sensorValues[4] > 200 && sensorValues[5] > 200 && sensorValues[6] > 200)
		{
			//Serial.print("90 Detected");
			turn('L',SPEED_TURN,100);
			path[pathCounter++] = 'L';
			BLUETOOTH.print(path[pathCounter - 1]);
		}
		else
		{
			turn('L', SPEED_TURN, 100);
			path[pathCounter++] = 'J';
			BLUETOOTH.print(path[pathCounter - 1]);
		}
		BUZZER_OFF;
	}

	// Right 90 Turn
	else if (sensorValues[4] < 200 && sensorValues[5] < 200 && sensorValues[6] < 200 && sensorValues[7] < 200)
	{

		while (sensorValues[4] < 200 && sensorValues[5] < 200 && sensorValues[6] < 200 && sensorValues[7] < 200)
		{
			BUZZER_ON;
			position_ = qtrRC.readLine(sensorValues, QTR_EMITTERS_ON, true);
		}
		if (sensorValues[1] > 200 && sensorValues[2] > 200 && sensorValues[3] > 200 && sensorValues[4] > 200 && sensorValues[5] > 200 && sensorValues[6] > 200)
		{
			//Serial.print("90 Detected");
			turn('R', SPEED_TURN, 100);
			path[pathCounter++] = 'R';
			BLUETOOTH.print(path[pathCounter - 1]);
		}
		else
		{
			turn('R', SPEED_TURN, 100);
			path[pathCounter++] = 'J';
			BLUETOOTH.print(path[pathCounter - 1]);

		}
		BUZZER_OFF;
	}
}


void loop()
{

	//delay(500);
	if (WHITE_ON_BLACK == 1)
		position_ = qtrRC.readLine(sensorValues, QTR_EMITTERS_ON, true);

	else
		position_ = qtrRC.readLine(sensorValues);

	runMappingMode();

	if (DEBUG_MODE == 1)
		showSensorValues();

	proportional = position_ - 3500;

	derivative = proportional - lastProportional;
	lastProportional = proportional;
	integral += proportional;

	control = proportional *KP + integral*KI + derivative*KD;

	if (DEBUG_MODE == 2)
	{
		Serial.print("Control  ");
		Serial.print(control);
		Serial.print(" Position  ");
		Serial.print(proportional);
		Serial.println("  ");
	}


	if (control > SPEED_MAX)
		control = SPEED_MAX;
	if (control < -SPEED_MAX)
		control = -SPEED_MAX;

	if (control < 0)
	{
		motorLeft.write(SPEED_MAX + control);
		motorRight.write(SPEED_MAX);
	}
	else
	{
		motorLeft.write(SPEED_MAX);
		motorRight.write(SPEED_MAX - control);
	}
}
