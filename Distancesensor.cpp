/*
 * DistanceSensor.cpp
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

HAL_ADC Dsensor(ADC_IDX1); /* declare HAL_ADC for ADC_IDX1, please refer to hal_adc.h for correct ADC mapping */



class DistanceSensor: public Thread {

	float start=0;
	double orbitalperiod=0;
	double thresholdtarget = 20;
	bool away = false;
	bool away1 = false;
	bool started = false;
	bool triggered1 = false;
	bool triggered2 = false;
	int counter = 0;
	int countergoal;
	int goal = 2;
	bool found = false;
	bool starting = false;
	bool measuring = false;
	int mode = 0;
	double distance;
	double distancesum;
	double distancemax;
	float Voltage1;
	float Voltage;

public:

	DistanceSensor(const char* name) : Thread(name) {
	}

	void init() {

		Dsensor.config(ADC_PARAMETER_RESOLUTION, 12);
		Dsensor.init(SharpSensorADC); /* initialization of the HAL object should be called one time only in the project*/
	}

	void run() {
		this->setPeriodicBeat(0, 1*MILLISECONDS);
		counter = 0;
		goal = 2;
		thresholdtarget = 35;
		distancesum = 0;
		distancemax = 0;
		countergoal= 10;
		while (1) {
			counter++;
			int16_t Value = Dsensor.read(SharpSensorADC);
			Voltage=((Value*3.0)/4096);
			Voltage1=Voltage*(2.6/73);
			Voltage1=1/Voltage1;
			distance=((0.348/Voltage)-0.0688)*100;
			if(distance > distancemax){
				distancemax = distance;
			}
			distancesum += distance;		
			if(counter == countergoal){
				counter = 0;
				distance = (distancesum - distancemax) / ((double)countergoal - 1.0);
				DistanceTopic.publish(distance);				
				distancesum = 0;
				distancemax = 0;
			}
			suspendUntilNextBeat();
		}
	}
};
DistanceSensor DistanceSensor("DistanceSensor");
