/*
 * SunFinder.cpp
 *
 *  Created on: 18.12.2017
 *      Author: I8FL-PC01-G01
 */
#include "rodos.h"
#include <stdio.h>
#include "hal.h"
#include "math.h"
#include "myStructs.h"
#include "myTopics.h"

class SunFinder: public Thread {
	double startposition;
	double maxLightValue;
	double maxLightPos;
	double currentPos;
	double currentValue;
	double currentRPM;
	double speed;
	Angle position;
	int mode;
	int suncounter;
	int suncountergoal;
	Controllerstatus controllerstatus;

public:

	SunFinder(const char* name) : Thread(name) {
	}

	void init() {

	}

	void run() {
		this->setPeriodicBeat(0, SENSE_SAMPLE_TIME*MILLISECONDS);
		mode = 0;
		ModeTopic.publish(mode);
		speed = 30;
		suncounter = 0;
		suncountergoal = 80;

		while (1) {
			ModeBuffer.get(mode);
			if (mode == 1){
				speed = 30;
				PRINTF("JJSunfinding initialized\n");
				RPYTopicBuffer.get(position);
				startposition = position.yaw;
				maxLightValue = 0;
				maxLightPos = startposition;
				SpeedTargetTopic.publish(speed);
				MotorTopic.publish(mode);
				int controllerready = 0;
				controllerstatus.RPM = 1;
				controllerstatus.speed = 1;
				controllerstatus.position = 0;
				ControllerstatusTopic.publish(controllerstatus);
				PRINTF("JJSunfinding in progress...\n");
				for ( int i = 0; i <= 1500; i++){
					SolarBuffer.get(currentValue);
					if(currentValue > maxLightValue){
						RPYTopicBuffer.get(position);
						maxLightValue = currentValue;
						maxLightPos = position.yaw;
					}
					suspendCallerUntil(NOW()+ 10 * MILLISECONDS);
				}
				controllerstatus.RPM = 1;
				controllerstatus.speed = 1;
				controllerstatus.position = 0;
				ControllerstatusTopic.publish(controllerstatus);
				speed = 0;
				SpeedTargetTopic.publish(speed);
				suspendCallerUntil(NOW()+ 2000 * MILLISECONDS);
				RPMBuffer.get(currentRPM);
				RPMSteadyTopic.publish(currentRPM);
				PRINTF("JJMax Solar Value at %f degrees, now approaching that position...\n", maxLightPos);
				ControllerTargetTopic.publish(maxLightPos);
				controllerstatus.RPM = 1;
				controllerstatus.speed = 1;
				controllerstatus.position = 1;
				ControllerstatusTopic.publish(controllerstatus);
				MotorTopic.publish(mode);
				RPYTopicBuffer.get(position);
				suncounter = 0;
				while(suncounter < suncountergoal){
					RPYTopicBuffer.get(position);
					if(betrag(position.yaw - maxLightPos) < 10){
						suncounter++;
						PRINTF("JJCounter = %d \n", suncounter);
					}
					else{
						suncounter = 0;
					}
					suspendCallerUntil(NOW()+ 50 * MILLISECONDS);
				}
				PRINTF("JJLooking at sun, now deploying Solar Panels.\n");
				suspendCallerUntil(NOW()+ 1000 * MILLISECONDS);
				mode = 2;
				ModeTopic.publish(mode);
				bool knife = true;
				ThermalKnifeTopic.publish(knife);
				suspendCallerUntil(NOW()+ 22 * SECONDS); //TODO Smart Deployment Detection
				PRINTF("JJThermal Knife turned off.\n");
				knife = false;
				ThermalKnifeTopic.publish(knife);
				mode = 3;
				ModeTopic.publish(mode);
				//controllerstatus.RPM =0;
				//controllerstatus.speed = 0;
				//controllerstatus.position = 0;
				//ControllerstatusTopic.publish(controllerstatus);
				//double motorspeed = 0;
				//RPMTargetTopic.publish(motorspeed);
				//mode = 3;
				//ModeTopic.publish(mode);

			}
			suspendCallerUntil(NOW()+ 500 * MILLISECONDS);

		}
	}
};
SunFinder SunFinder("SunFinder");
