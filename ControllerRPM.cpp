/*
/*
 * Controller.cpp
 *
 *  Created on: 14.01.2018
 *      Author: I8FL-PC01-G01
 */

#include "rodos.h"
#include <stdio.h>
#include "hal.h"
#include "math.h"
#include "stdint.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_misc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "myTopics.h"
#include "myStructs.h"

#define BUTTON GPIO_058
#define ADDR 0x39 /*Slave address*/




class ControllerRPM: public Thread {

private:
	double RPMcurrent;
	double RPMdesired;
	double errorP;
	double errorI;
	double errorD;
	double errorlast;
	double errortotal;
	PID pid;
	double dutycycle;
	Winkel RPY;
	int counter = 0;
	int countergoal;
	int controllerready;
	IMU_Data imudata;
	double currentspeed;
	double desiredspeed;
	Controllerstatus controllerstatus;

public:

	ControllerRPM(const char* name) : Thread(name) {
	}

	void init() {
		RPMdesired = 0;
		errorI = 0;
		errorlast = 0;
		errortotal = 0;
		controllerready = 1;
		ErrorRPMTopic.publish(errortotal);
		desiredspeed = 0;
		SpeedTargetTopic.publish(desiredspeed);
	}

	void run() {
		countergoal = 10;
		pid.P = 1.5; //1.5
		pid.I = 0.035; //0.035
		//pid.D = 1;
		pid.D = 0.5; //0.5
		this->setPeriodicBeat(0, SENSE_SAMPLE_TIME * MILLISECONDS);
		RPMControllerPIDTopic.publish(pid);
		while (1) {
			ControllerstatusBuffer.get(controllerstatus);
			RPMControllerPIDBuffer.get(pid);
			if(controllerstatus.RPM == 1){
				counter++;
				RPMBuffer.get(RPMcurrent);
				RPMTargetBuffer.get(RPMdesired);
				errorP =  RPMdesired - RPMcurrent;
				errorI = errorI + errorP;
				if (errorI > 10000){
					errorI = 10000;
				}
				else if(errorI < -10000){
					errorI = -10000;
				}
				errorD = errorP - errorlast;				
				errortotal = pid.P * errorP+ pid.D * errorD + pid.I * errorI;			
				dutycycle = errortotal;
				if (counter == countergoal){
					counter = 0;					
				}
				
				ErrorRPMTopic.publish(dutycycle);
				errorlast = errorP;
			}
			else if(controllerstatus.RPM == 2){
				counter++;
				SpeedTargetBuffer.get(desiredspeed);
				IMUBuffer.get(imudata);
				currentspeed = imudata.gyroData[2];
				RPMBuffer.get(RPMcurrent);/
				errorP = desiredspeed-currentspeed;
				errorI = errorI + errorP;
				errorD = errorP - errorlast;
				errortotal =pid.P * errorP + pid.D * errorD + pid.I * errorI;
				dutycycle = errortotal;
				if (counter == countergoal){
					counter = 0;
					PRINTF("JJcurrent RPM: %f\n", RPMcurrent);
					PRINTF("JJdesired RPM: %f\n", RPMdesired);
					PRINTF("JJcurrent Speed: %f\n", currentspeed);
					PRINTF("JJdesired Speed: %f\n", desiredspeed);
					PRINTF("JJerrorP: %f\n", errorP);
					PRINTF("JJdutycycle: %f\n", dutycycle);
				}
				ErrorRPMTopic.publish(dutycycle);
				errorlast = errorP;
			}
			else{
				errorI = 0;
				errorlast = 0;
				errortotal = 0;
			}
			suspendUntilNextBeat();
		}

	}
};
ControllerRPM ControllerRPM("ControllerRPM");






