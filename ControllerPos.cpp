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




class ControllerPos: public Thread {

private:
	double yawcurrent;
	double yawdesired;
	double error;
	double errorI;
	double errorD;
	double errorP;
	double errorlast;
	double errortotal;
	double currentRPM;
	double desiredspeed;
	PID pid;
	Winkel RPY;
	int counter = 0;
	int countergoal = 3;
	int controllerready = 0;
	Controllerstatus controllerstatus;

public:

	ControllerPos(const char* name) : Thread(name) {
	}

	void init() {
		yawdesired = 0;
		errorI = 0;
		errorlast = 0;
		errortotal = 0;
		controllerready = 0;
	}

	void run() {

		this->setPeriodicBeat(0, SENSE_SAMPLE_TIME  * MILLISECONDS);
		// CHANGE PID VALUES HERE
		pid.P = 0.5;
		pid.I = 0.0018;
		pid.D = 0.05; 

		//ControllerPIDTopic.publish(pid);
		while (1) {
			ControllerstatusBuffer.get(controllerstatus);
			//ControllerPIDBuffer.get(pid);
			if(controllerstatus.position==1){
				ControllerTargetBuffer.get(yawdesired);
				counter++;
				RPYTopicBuffer.get(RPY);
				yawcurrent = RPY.yaw;
				error =  yawdesired - yawcurrent;
				RPMBuffer.get(currentRPM);
				if (error > 180){
					error -= 360;
				}
				else if(error < -180){
					error += 360;
				}
				error = saturate(error, 90);
				errorI = errorI + saturate(error,30);
				errorD = (error - errorlast);
				errortotal =  pid.P * error + pid.D * errorD + pid.I * errorI;
				desiredspeed =  -errortotal;
				if (counter == countergoal){
					counter = 0;					
				}
				SpeedTargetTopic.publish(desiredspeed);
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
ControllerPos ControllerPos("ControllerPos");






