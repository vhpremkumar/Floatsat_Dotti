/*
 * Tracker.cpp
 *
*  Created on: 18.12.2017
 *      Author: I8FL-PC01-G01
 *
 */
#include "rodos.h"
#include <stdio.h>
#include "hal.h"
#include "math.h"
#include "myStructs.h"
#include "myTopics.h"

class Tracker: public Thread {

	Controllerstatus controllerstatus;
	double startposition;
	double maxLightValue;
	double maxLightPos;
	double currentPos;
	double currentValue;
	double speed;
	Angle position;
	int mode;
	int counter;
	int countergoal;
	double orbitalperiod;
	double targetspeed;
	double findingspeed;
	double desireddistance;
	double currentdistance;
	double currentspeed;
	double error;
	double errorlast;
	double errorI;
	double iterator;
	double Pfactor;
	double Dfactor;
	double Ifactor;
	double desiredspeed;
	IMU_Data imudata;
	PID pid;

public:

	Tracker(const char* name) : Thread(name) {
	}

	void init() {

	}

	void run() {
		this->setPeriodicBeat(0, SENSE_SAMPLE_TIME*MILLISECONDS);
		mode = 0;
		ModeTopic.publish(mode);
		speed = 30;
		counter = 0;
		countergoal = 30;
		pid.P = 1.6;
		pid.D = 0.02;
		pid.I = 0.02;
		ControllerPIDTopic.publish(pid);

		while (1) {
			ModeBuffer.get(mode);
			if (mode == 6){
				counter++;
				OrbitalPeriodBuffer.get(orbitalperiod);
				targetspeed = 360.0 / orbitalperiod;
				findingspeed= targetspeed;
				SpeedTargetTopic.publish(targetspeed);
				controllerstatus.RPM = 1;
				controllerstatus.speed = 1;
				controllerstatus.position = 0;
				int data1 = 1;
				MotorTopic.publish(data1);
				mode = 7;
				ControllerstatusTopic.publish(controllerstatus);
				ModeTopic.publish(mode);
				desireddistance=24.5; 
				PRINTF("JJJJNow accelerating to %f degrees per second \n",targetspeed);
			}
			else if(mode == 7){
				IMUBuffer.get(imudata);
				currentspeed = imudata.gyroData[2];
				DistanceBuffer.get(currentdistance);
				if (currentdistance > 26.5){ 
					findingspeed = findingspeed + 0.01; //TUNING
					SpeedTargetTopic.publish(findingspeed);
					counter++;
					if (counter == countergoal){
						PRINTF("JJIncreasing speed to %f degrees per second \n",findingspeed);
					counter = 0;
					}
					iterator = 0;
				}
				else{
					iterator++;
					if(iterator == 5){
						mode = 8;
						ModeTopic.publish(mode);
						//targetspeed -= 2;
						SpeedTargetTopic.publish(targetspeed);
						PRINTF("JJTarget Found! Controller starting \n");
						countergoal = 10;
						counter = 0;
						errorlast = 0;
						errorI = 0;
					}
				}
			}
			else if(mode == 8){
				counter++;
				DistanceBuffer.get(currentdistance);
				error = desireddistance - currentdistance;
				ControllerPIDBuffer.get(pid);
				IMUBuffer.get(imudata);
				currentspeed = imudata.gyroData[2];
				errorI = errorI + error;
				desiredspeed = targetspeed - pid.P * error - (pid.D * (error - errorlast)) - pid.I * errorI;
				errorlast = error;
				SpeedTargetTopic.publish(desiredspeed);
				if (counter == countergoal){
					PRINTF("JJError = %f \n",error);				
					PRINTF("JJdesiredspeed = %f  \n",desiredspeed);
					counter = 0;
				}
			}
			suspendUntilNextBeat();
		}
	}
};
Tracker Tracker("Tracker");
