/*
 * Telemetry.cpp
 *
*  Created on: 18.12.2017
 *      Author: I8FL-PC01-G01
 *
 */

#include "rodos.h"
#include <stdio.h>
#include "hal.h"
#include "math.h"
#include "myTopics.h"
#include "myStructs.h"


extern bool telemetry_ready;

class Telemetry: public Thread {

public:

	Angle angles;
	double distanceValue;
	double SolarValue;
	TM tm;
	IMU_Data imudata;
	Battery batterydata;
	double Orbitperiod;
	int mode;
	double RPM;
	int GSmode;

	Telemetry(const char* name) : Thread(name) {
	}

	void init() {
		tm.sendRPY = false;
		tm.sendDistance = false;
		tm.sendLight = false;
		tm.sendIMU = false;
		tm.sendBattery = false;
		GSmode = 1;
		
	}

	void run() {
		TMTopic.publish(tm);
		while (1) {
			TMTopicBuffer.get(tm);
			if(GSmode == 0){
				if (tm.sendRPY){
					RPYTopicBuffer.get(angles);
					IMUBuffer.get(imudata);
					PRINTF("Roll: %f, Pitch: %f, Yaw: %f \n", angles.roll, angles.pitch, angles.yaw);
					PRINTF("Current Speed: %f degrees per second \n",imudata.gyroData[2]);
				}
				if (tm.sendDistance){
					DistanceBuffer.get(distanceValue);
					PRINTF("Distance Sensor Value: %f cm\n",distanceValue);
				}
				if (tm.sendLight){
					SolarBuffer.get(SolarValue);
					PRINTF("Total light sensor reading: %f \n",SolarValue);
				}
				if (tm.sendBattery){
					BatteryBuffer.get(batterydata);
					PRINTF("Battery Current: %f A \n",batterydata.I);
					PRINTF("Battery Voltage: %f V \n",batterydata.U);
					PRINTF("Battery Charge: %f Percent \n",batterydata.C);
					PRINTF("Solarpanel Power: %f mW\n\n",batterydata.SolarP);

				}
			}
			else if(GSmode == 1){
				ModeBuffer.get(mode);
				RPMBuffer.get(RPM);
				BatteryBuffer.get(batterydata);
				DistanceBuffer.get(distanceValue);
				IMUBuffer.get(imudata);
				RPYTopicBuffer.get(angles);
				SolarBuffer.get(SolarValue);
				PRINTF("AA%f \n",angles.yaw);
				PRINTF("BB%f \n",imudata.gyroData[2]);
				PRINTF("CC%f \n",distanceValue);
				PRINTF("DD%f \n",batterydata.C);
				PRINTF("EE%d \n",mode);
				PRINTF("FF%f \n",batterydata.U);
				PRINTF("GG%f \n",batterydata.I);
				PRINTF("HH%f \n",batterydata.SolarP);
				PRINTF("II%f \n",RPM);
			}
            suspendCallerUntil(NOW()+150*MILLISECONDS);
		}
	}
};
Telemetry Telemetry("Telemetry");

/***********************************************************************/
