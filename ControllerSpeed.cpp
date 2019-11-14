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




class ControllerSpeed: public Thread {

private:
	double speedcurrent;
	double speeddesired;
	double RPMdesired;
	double RPMcurrent;
	double error;
	double errorI;
	double errorD;
	double errorP;
	double errorlast;
	double errortotal;
	PID pid;
	IMU_Data imudata;
	int counter = 0;
	int countergoal = 5;
	int controllerready = 0;
	Controllerstatus controllerstatus;

public:

	ControllerSpeed(const char* name) : Thread(name) {
	}

	void init() {
		speeddesired = 0;
		errorI = 0;
		errorlast = 0;
		errortotal = 0;
		controllerready = 0;		
	}

	void run() {

		this->setPeriodicBeat(0, SENSE_SAMPLE_TIME  * MILLISECONDS);
		// CHANGE PID VALUES HERE
		pid.P = 1.3;
		pid.I = 0.03; // 0.001
		pid.D = 8; // 0.003
		//ControllerPIDTopic.publish(pid);

		while (1) {
			ControllerstatusBuffer.get(controllerstatus);
		//	ControllerPIDBuffer.get(pid);
			if(controllerstatus.speed == 1){
				SpeedTargetBuffer.get(speeddesired);
				IMUBuffer.get(imudata);
				RPMBuffer.get(RPMcurrent);
				speedcurrent = imudata.gyroData[2];				
				error =  pid.D*(speeddesired - speedcurrent);
				errorP = error + speeddesired / 10;
				errorI = errorI + errorP;
				errorD = (errorP - errorlast);
				errortotal = pid.P * errorP + 0 * errorD + pid.I * errorI;
				RPMdesired = RPMcurrent + errortotal;				
				RPMTargetTopic.publish(RPMdesired);
				errorlast = error;
			}
			else{
				errorlast= 0;
				errorI = 0;
			}
			suspendUntilNextBeat();
		}

	}
};
ControllerSpeed ControllerSpeed("ControllerSpeed");






