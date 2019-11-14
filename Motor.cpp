/*
 * Motor.cpp
 *
 *  Created on: 14.01.2018
 *      Author: I8FL-PC01-G01
 */

#include "rodos.h"
#include <stdio.h>
#include "hal.h"
#include "math.h"
#include "stdint.h"
#include "myTopics.h"
#include "myStructs.h"

HAL_GPIO HBRIDGE_A_INA(GPIO_036); /* declare HAL_GPIO for GPIO_036 = PC4 (HBRIDGE-A INA pin) */
HAL_GPIO HBRIDGE_A_INB(GPIO_017); /* declare HAL_GPIO for GPIO_017 = PB1 (HBRIDGE-B INA pin) */
HAL_GPIO HBRIDGE_EN(GPIO_066); /* declare HAL_GPIO for GPIO_066 = PE2 (HBRIDGE Power Enable pin) */
HAL_PWM Motor(PWM_IDX12); /* declare HAL_PWM for PWM_IDX12 = TIM4-CH1 (HBRIDGE-A), please refer to hal_pwm.h for correct PWM mapping*/

double mspeedcurrent;
double mspeeddesired;

class Motorcontrol: public Thread {

private:
	TM tm;
	int mode;
	double requestedDutyCycle;
	int counter = 0;
	int countergoal = 100;
	unsigned int speed;

public:

	Motorcontrol(const char* name) :
			Thread(name) {
	}

	void init() {

		Motor.init(5000, 1000); /* initialization of the HAL object should be called one time only in the project*/
		HBRIDGE_EN.init(true, 1, 1); /* initialization of the HAL object should be called one time only in the project*/
		HBRIDGE_A_INA.init(true, 1, 0); /* initialization of the HAL object should be called one time only in the project*/
		HBRIDGE_A_INB.init(true, 1, 0); /* initialization of the HAL object should be called one time only in the project*/
		Motor.write(0);
		mspeedcurrent = 0;
		mode = 0;

	}

	void run() {

		//EncoderInit();
		this->setPeriodicBeat(0, SENSE_SAMPLE_TIME * MILLISECONDS);
		MotorTopic.publish(mode);
		mode = 0;

		while (1) {
			counter++;
			MotorBuffer.get(mode);
			if (mode == 0) {
				mspeeddesired=0;
			}
			else if (mode == 1){
				ErrorRPMBuffer.get(requestedDutyCycle);
				mspeeddesired = requestedDutyCycle;

			}
			if (mspeeddesired < 0) {
				HBRIDGE_A_INA.setPins(false);
				HBRIDGE_A_INB.setPins(true);
				if (mspeeddesired <= -1000) {
					mspeeddesired = -1000;
				}
				speed = (unsigned int) -mspeeddesired;
				Motor.write(speed);	
			}
			else {
				HBRIDGE_A_INA.setPins(true);
				HBRIDGE_A_INB.setPins(false);
				if (mspeeddesired >= 1000) {
					mspeeddesired = 1000;
				}				
				speed = (unsigned int) mspeeddesired;			
				Motor.write(speed);	 
			}

			if(counter == 1000*countergoal){
				counter = 0;
				PRINTF("JJmspeeddesired is %f\n", mspeeddesired);
				PRINTF("JJspeed is %d\n", speed);
			}
			mspeedcurrent = mspeeddesired;
			suspendUntilNextBeat();
		}
	}
};
Motorcontrol Motorcontrol("Motorcontrol");

