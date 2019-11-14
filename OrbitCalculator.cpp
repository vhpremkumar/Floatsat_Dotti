/*
 * OrbitCalculator.cpp
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

#define SharpSensorADC ADC_CH_001 // PA1 pin

class OrbitCalculator: public Thread {

	float start=0;
	double orbitalperiod=0;
	double thresholdtarget = 20;
	bool away = false;
	bool away1 = false;
	bool away2 = false;
	bool away3 = false;
	bool started = false;
	bool triggered1 = false;
	bool triggered2 = false;
	bool triggered3 = false;
	bool triggered4 = false;
	int counter = 0;
	int goal = 2;
	bool found = false;
	bool starting = false;
	bool measuring = false;
	int mode = 0;
	double distance;
	Winkel angles;
	double startyaw;

public:

	OrbitCalculator(const char* name) : Thread(name) {
	}

	void init() {

	}

	void run() {
		this->setPeriodicBeat(0, SENSE_SAMPLE_TIME*MILLISECONDS);
		counter = 0;
		goal = 3;
		thresholdtarget = 35;
		while (1) {
			ModeBuffer.get(mode);
			if(mode == 4){
				PRINTF("JJStabalizing for Orbital Measurement\n");
				mode = 5;
				ModeTopic.publish(mode);
				measuring = true;
				away = false;
				away1 = false;
				away2 = false;
				away3 = false;
				RPYTopicBuffer.get(angles);
				startyaw = angles.yaw;
				started = false;
				triggered1 = false;
				triggered2 = false;
				triggered3 = false;
				triggered4 = false;
				Controllerstatus controllerstatus;
				controllerstatus.position = 1;
				controllerstatus.RPM = 1;
				controllerstatus.speed = 1;
				ControllerTargetTopic.publish(startyaw);
				ControllerstatusTopic.publish(controllerstatus);
				int data1 = 1;
				MotorTopic.publish(data1);
				counter = 0;
				found = false;
				suspendCallerUntil(NOW()+ 5 * SECONDS);
				PRINTF("JJOrbitalPeriod Calculation initialized\n");
			}
			if(measuring){
				DistanceBuffer.get(distance);
				if(thresholdtarget>=distance && found == false){
					if (away&&triggered3){
						counter++;
						away = false;
						away1 = false;
						away2 = false;
						away3 = false;
						PRINTF("JJCounter = %d \n",counter);
						if(counter == goal){
							orbitalperiod = (SECONDS_NOW() - start) / (double) goal;
							OrbitalPeriodTopic.publish(orbitalperiod);
							PRINTF("JJOrbitalPeriod = %f \n",orbitalperiod);
							found=true;
							measuring = false;
							//int seconds = round_s(orbitalperiod/ * (double)SECONDS);
							suspendCallerUntil(NOW()+ 500 * MILLISECONDS);
							mode = 6;
							ModeTopic.publish(mode);
						}
					}
					if (triggered3){
						if (!started){
							start=SECONDS_NOW();
							started = true;
							PRINTF("JJStarted counting\n");
						}
						triggered4 = true;
					}
					if (triggered2){
						triggered3 = true;
					}
					if (triggered1){
						triggered2 = true;
					}
					triggered1 = true;
				}
				else{
					triggered1 = false;
					triggered2 = false;
					triggered3 = false;
				}
				if (triggered4 && found == false){
					if(distance >= thresholdtarget && away1){
						away = true;
						triggered4=false;
						triggered3=false;
						triggered2=false;
						triggered1=false;
					}
					if (away2){
						away1 = true;
					}
					if (away3){
						away2 = true;
					}
					away3 = true;
				}
				else{
					away1=false;
					away2=false;
					away3=false;
				}
			}
			suspendUntilNextBeat();
		}
	}
};
OrbitCalculator OrbitCalculator("OrbitCalculator");
