/*
 * IMU.cpp
 *
 *  Created on: 18.12.2017
 *      Author: I8FL-PC01-G01
 */

#include "rodos.h"
#include <stdio.h>
#include "hal.h"
#include "math.h"
#include "matlib.h"
#include "myTopics.h"
#include "myStructs.h"





//HAL_I2C imu(I2C_IDX2);
HAL_GPIO imu_EN(GPIO_055);	// PD7, IMU Power Enable
HAL_GPIO cs_G(GPIO_018);	// PB2, Gyro Chip Select
HAL_GPIO cs_XM(GPIO_032);	// PC0, XM Chip Select
HAL_SPI imu(SPI_IDX1);
//HAL_UART BT2UART(UART_IDX2);


#define LSM9DS0_G				0x6B            // Gyro 7-bit I2C address
#define LSM9DS0_XM				0x1D			// IMU 7-bit I2C address

#define OUT_X_L_G				0x28
#define OUT_X_L_A				0x28
#define OUT_X_L_M				0x08
#define OUT_TEMP_L_XM			0x05

#define MSB						0x80

#define DPS_MAX					2000
#define ACC_MAX					2
#define MAG_MAX					2

#define DPS_DOUBLE(x)			((double)x*DPS_MAX/INT16_MAX)
#define ACC_DOUBLE(x)			((double)x*ACC_MAX/INT16_MAX)
#define TEMP_DOUBLE(x)			((double)x/4)
#define MAG_DOUBLE(x)			((double)x*MAG_MAX/INT16_MAX)
#define MAG_NORMALIZE(x, min, max)	((MAG_DOUBLE(x)-MAG_DOUBLE(min))/(MAG_DOUBLE(max)-MAG_DOUBLE(min))*2-1)


uint8_t LSM9DS0_G_DATA[1] = {OUT_X_L_G | MSB | 0x40};
uint8_t LSM9DS0_A_DATA[1] = {OUT_X_L_A | MSB | 0x40};
uint8_t LSM9DS0_M_DATA[1] = {OUT_X_L_M | MSB | 0x40};
uint8_t LSM9DS0_T_DATA[1] = {OUT_TEMP_L_XM | MSB | 0x40};

//uint8_t LSM9DS0_WHO_AM_I_G[1] = { 0x0F | 0x80 }; /* Gyro WHO_AM_I register address*/

uint8_t CTRL_REG1_G[2] = { 0x20, 0x4F };
uint8_t CTRL_REG4_G[2] = { 0x23, 0xA0 };
uint8_t CTRL_REG_XM[9] ={0x1f | 0x40 , 0x00, 0x7f, 0xc0, 0x00, 0x00, 0x94, 0x00, 0x80};

TM ready;
bool calibrate = false;

namespace RODOS
{
extern HAL_UART uart_stdout;
}

#define TeleUART uart_stdout

struct SensorData
{
	double gyro_x, gyro_y, gyro_z;
	double acc_x, acc_y, acc_z;
	double mag_x, mag_y, mag_z;
	double pitchAccMag, rollAccMag, headingAccMag;
	double pitchGyro, rollGyro, headingGyro;
	double pitch, roll, heading;
	double temp;
};

struct TelecommandData
{
	char msg_id;
	char msg[11];
	char *msg_end;
};

class SignalProc: public Thread
{
	double gyro_offx;
	double gyro_offy;
	double gyro_offz;
	double acc_offx;
	double acc_offy;
	double acc_offz;
	int state;
	int16_t min_x, max_x, min_y, max_y, min_z, max_z;

	double pitchAccMag, rollAccMag, headingAccMag;
	double pitchGyro, rollGyro, headingGyro;
	double pitch, roll, heading;

	double T;

	double alpha;

	SensorData sd;

	Quaternion Q;
	YPR ypr;

public:

	SignalProc(const char* name) :
			Thread(name)
	{

		gyro_offx = 2.748;
		gyro_offy = -0.535;
		gyro_offz = -2.660;
		acc_offx = -0.129;
		acc_offy = -0.036;
		acc_offz = -0.029;
		min_x = -6786;
		max_x = 3528;
		min_y = -4046;
		max_y = 4109;
		min_z = -2586;
		max_z = 4596;

		pitchAccMag = 0;
		rollAccMag = 0;
		headingAccMag = 0;
		pitchGyro = 0;
		rollGyro = 0;
		headingGyro = 0;
		pitch = 0;
		roll = 0;
		heading = 0;

		T = 10;
		alpha = 0.9;
		state = 3;
	}

	void init()
	{
		imu.init();
		imu_EN.init(true, 1, 1);
		cs_G.init(true, 1, 1);
		cs_XM.init(true, 1, 1);
		ready.sendRPY = false;
		ready.sendBattery = false;
		ready.sendDistance = false;
		ready.sendIMU = false;
		ready.sendLight = false;		
	}

	void imuReset()
	{
		imu_EN.setPins(0);
		imu.reset();

		suspendCallerUntil(NOW()+ 10 * MILLISECONDS);

		imu.init(400000);
		imu_EN.setPins(1);
	}

	void imuInit()
	{
		uint8_t xff = 0xFF;

		cs_G.setPins(0);
		imu.write(CTRL_REG1_G, 2);
		cs_G.setPins(1);
		imu.write(&xff, 1);

		cs_G.setPins(0);
		imu.write(CTRL_REG4_G, 2);
		cs_G.setPins(1);
		imu.write(&xff, 1);

		cs_XM.setPins(0);
		imu.write(CTRL_REG_XM, 9);
		cs_XM.setPins(1);
		imu.write(&xff, 1);


	}


	void read(HAL_GPIO &pin, uint8_t *targetregister, int16_t *dataarray, int amountofbits)
{
		pin.setPins(0);
		imu.write(targetregister, 1);
		imu.read((uint8_t*)dataarray , amountofbits);		
		pin.setPins(1);		
	}

	void calibrateGyro()
	{
		PRINTF("JJGyro Calibration\nPut the board on still ground and enter $Y,1,#\n");
		CalibrationBuffer.get(calibrate);
		while (calibrate == false){
			CalibrationBuffer.get(calibrate);
			suspendCallerUntil(NOW() + 10000 * NANOSECONDS);
		}
		calibrate = false;
		CalibrationTopic.publish(calibrate);
		PRINTF("JJCalibrating...\n");
		double value[3] = {0.0};
		int16_t data[3];
		for (int i = 0; i < 10000; i++)
		{
			read(cs_G, LSM9DS0_G_DATA, data, 6);		
			value[0] += DPS_DOUBLE(data[0]);
			value[1] += DPS_DOUBLE(data[1]);
			value[2] += DPS_DOUBLE(data[2]);		
		}
		gyro_offx = value[0] / 10000;
		gyro_offy = value[1] / 10000;
		gyro_offz = value[2] / 10000;

		PRINTF("JJCalibration finished.\n");
		PRINTF("JJOffset X: %f, Y: %f, Z: %f \n\n", gyro_offx, gyro_offy, gyro_offz);

		pitch = 0;
		roll = 0;
		heading = 0;

		PRINTF("JJCalibration finished\nEnter $Y,1,# to start measuring.\n\n");
		CalibrationBuffer.get(calibrate);
		while (calibrate == false){
			CalibrationBuffer.get(calibrate);
			suspendCallerUntil(NOW() + 10000 * NANOSECONDS);
		}
		calibrate = false;
		CalibrationTopic.publish(calibrate);
		ready.sendRPY = true;		
		state = 3;
	}

	void calibrateAcc()
	{
		double value[3] = {0.0};
		int16_t data[3];

		PRINTF("JJAcc Calibration\nLet the x-axis point to the sky, then enter $Y,1,#\n");
		CalibrationBuffer.get(calibrate);
		while (calibrate == false){
			CalibrationBuffer.get(calibrate);
			suspendCallerUntil(NOW() + 10000 * NANOSECONDS);
		}
		calibrate = false;
		CalibrationTopic.publish(calibrate);
		PRINTF("JJCalibrating...\n");
		for (int i = 0; i < 10000; i++)
		{
			read(cs_XM, LSM9DS0_A_DATA, data, 6);			
			value[1] += ACC_DOUBLE(data[1]);
			value[2] += ACC_DOUBLE(data[2]);			
		}

		PRINTF("JJLet the y-axis point to the sky, then enter $Y,1,#\n");
		CalibrationBuffer.get(calibrate);
		while (calibrate == false){
			CalibrationBuffer.get(calibrate);
			suspendCallerUntil(NOW() + 10000 * NANOSECONDS);
		}
		calibrate = false;
		CalibrationTopic.publish(calibrate);
		PRINTF("JJCalibrating...\n");
		for (int i = 0; i < 10000; i++)
		{
			read(cs_XM, LSM9DS0_A_DATA, data, 6);			/
			value[0] += ACC_DOUBLE(data[0]);
			value[2] += ACC_DOUBLE(data[2]);			
		}

		PRINTF("JJLet the z-axis point to the sky, then enter $Y,1,#\n");
		CalibrationBuffer.get(calibrate);
		while (calibrate == false){
			CalibrationBuffer.get(calibrate);
			suspendCallerUntil(NOW() + 10000 * NANOSECONDS);
		}
		calibrate = false;
		CalibrationTopic.publish(calibrate);
		PRINTF("JJCalibrating...\n");
		for (int i = 0; i < 10000; i++)
		{
			read(cs_XM, LSM9DS0_A_DATA, data, 6);			
			value[0] += ACC_DOUBLE(data[0]);
			value[1] += ACC_DOUBLE(data[1]);		
		}


		acc_offx = value[0] / 20000;
		acc_offy = value[1] / 20000;
		acc_offz = value[2] / 20000;

		PRINTF("JJCalibration finished.\n");
		PRINTF("JJOffset X: %f, Y: %f, Z: %f \n\n", acc_offx, acc_offy, acc_offz);

		pitch = 0;
		roll = 0;
		heading = 0;		
		calibrateGyro();
	}

	void calibrateMag()
	{
		min_x = INT16_MAX;
		max_x = 0;
		min_y = INT16_MAX;
		max_y = 0;
		min_z = INT16_MAX;
		max_z = 0;
		int16_t data[3];
		int counter = 0;
		int counter1 = 0;
		int countergoal = 2000;

		PRINTF("JJMag Calibration\nEnter $Y,1,#, then rotate around the x-Axis for 30 seconds (at least two rotations).\n");
		CalibrationBuffer.get(calibrate);
		while (calibrate == false){
			CalibrationBuffer.get(calibrate);
			suspendCallerUntil(NOW() + 10000 * NANOSECONDS);
		}
		calibrate = false;
		CalibrationTopic.publish(calibrate);
		PRINTF("JJCalibrating...\n");
		for (int i = 0; i < 300000; i++)
		{
			counter++;
			read(cs_XM, LSM9DS0_M_DATA, data, 6);
			if (data[0] < min_x) min_x = data[0];
			if (data[0] > max_x) max_x = data[0];
			if (data[1] < min_y) min_y = data[1];
			if (data[1] > max_y) max_y = data[1];
			if (data[2] < min_z) min_z = data[2];
			if (data[2] > max_z) max_z = data[2];			
			if (counter == 5*countergoal){
				counter = 0;
				counter1++;
				PRINTF("JJRotating for %d Seconds now...\n", counter1);
			}
		}
		PRINTF("JJMIN X: %d ", min_x);
		PRINTF("JJMAX X: %d \n", max_x);
		PRINTF("JJMIN Y: %d ", min_y);
		PRINTF("JJMAX Y: %d \n", max_y);
		PRINTF("JJMIN Z: %d ", min_z);
		PRINTF("JJMAX Z: %d \n\n", max_z);
		counter = 0;
		counter1 = 0;

		PRINTF("JJEnter $Y,1,#, then rotate around the y-Axis for 30 seconds (at least two rotations).\n");
		CalibrationBuffer.get(calibrate);
		while (calibrate == false){
			CalibrationBuffer.get(calibrate);
			suspendCallerUntil(NOW() + 10000 * NANOSECONDS);
		}
		calibrate = false;
		CalibrationTopic.publish(calibrate);
		PRINTF("JJCalibrating...\n");
		for (int i = 0; i < 300000; i++)
		{
			counter++;
			read(cs_XM, LSM9DS0_M_DATA, data, 6);			
			if (data[0] < min_x) min_x = data[0];
			if (data[0] > max_x) max_x = data[0];
			if (data[1] < min_y) min_y = data[1];
			if (data[1] > max_y) max_y = data[1];
			if (data[2] < min_z) min_z = data[2];
			if (data[2] > max_z) max_z = data[2];			
			if (counter == 5* countergoal){
				counter = 0;
				counter1++;
				PRINTF("JJRotating for %d Seconds now...\n", counter1);
			}
		}
		counter = 0;
		counter1 = 0;
		PRINTF("JJMIN X: %d ", min_x);
		PRINTF("JJMAX X: %d \n", max_x);
		PRINTF("JJMIN Y: %d ", min_y);
		PRINTF("JJMAX Y: %d \n", max_y);
		PRINTF("JJMIN Z: %d ", min_z);
		PRINTF("JJMAX Z: %d \n\n", max_z);

		PRINTF("JJEnter $Y,1,#, then rotate around the z-Axis for 30 seconds (at least two rotations).\n");
		CalibrationBuffer.get(calibrate);
		while (calibrate == false){
			CalibrationBuffer.get(calibrate);
			suspendCallerUntil(NOW() + 10000 * NANOSECONDS);
		}
		calibrate = false;
		CalibrationTopic.publish(calibrate);
		PRINTF("JJCalibrating...\n");
		for (int i = 0; i < 300000; i++)
		{
			counter++;
			read(cs_XM, LSM9DS0_M_DATA, data, 6);		
			if (data[0] < min_x) min_x = data[0];
			if (data[0] > max_x) max_x = data[0];
			if (data[1] < min_y) min_y = data[1];
			if (data[1] > max_y) max_y = data[1];
			if (data[2] < min_z) min_z = data[2];
			if (data[2] > max_z) max_z = data[2];		
			if (counter == 5* countergoal){
				counter = 0;
				counter1++;
				PRINTF("JJRotating for %d Seconds now...\n", counter1);
			}
		}
		counter = 0;
		counter1 = 0;

		PRINTF("JJCalibration finished.\n");
		PRINTF("JJMIN X: %d ", min_x);
		PRINTF("JJMAX X: %d \n", max_x);
		PRINTF("JJMIN Y: %d ", min_y);
		PRINTF("JJMAX Y: %d \n", max_y);
		PRINTF("JJMIN Z: %d ", min_z);
		PRINTF("JJMAX Z: %d \n\n", max_z);

		pitch = 0;
		roll = 0;
		heading = 0;		
		calibrateAcc();
	}


	void run()
	{
		calibrate = false;
		CalibrationTopic.publish(calibrate);

		int16_t data[3];
		int16_t imuData[3];
		int16_t magData[3];
		int16_t tempData;
		imuInit();
		switch(state) {
		case 0:
			calibrateMag();
			break;
		case 1:
			calibrateAcc();
			break;
		case 2:
			calibrateGyro();
			break;
		case 3:
			ready.sendRPY = true;
			break;
		}


		this->setPeriodicBeat(0, SENSE_SAMPLE_TIME*MILLISECONDS);
		while (state == 3)
		{
			int f = 0;
			if (f == 1)
			{
				calibrateGyro();
			}
			else if(f == 2)
			{
				calibrateAcc();
			}
			else if (f == 3)
			{
				calibrateMag();
			}

			read(cs_G, LSM9DS0_G_DATA, data, 6);
			read(cs_XM, LSM9DS0_A_DATA, imuData, 6);
			read(cs_XM, LSM9DS0_M_DATA, magData, 6);
			read(cs_XM, LSM9DS0_T_DATA, &tempData, 2);


			tempData = (tempData & 0x800) ? (-1 ^ 0xFFF) | tempData : tempData;

			sd.gyro_x = DPS_DOUBLE(data[0]) - gyro_offx;
			sd.gyro_y = DPS_DOUBLE(data[1]) - gyro_offy;
			sd.gyro_z = DPS_DOUBLE(data[2]) - gyro_offz;

			sd.acc_x = ACC_DOUBLE(imuData[0]) - acc_offx;
			sd.acc_y = ACC_DOUBLE(imuData[1]) - acc_offy;
			sd.acc_z = ACC_DOUBLE(imuData[2]) - acc_offz;


			sd.mag_x = MAG_NORMALIZE(magData[0], min_x, max_x);
			sd.mag_y = MAG_NORMALIZE(magData[1], min_y, max_y);
			sd.mag_z = MAG_NORMALIZE(magData[2], min_z, max_z);

			sd.temp = TEMP_DOUBLE(tempData);
			
			IMU_Data	imu_new;
			imu_new.gyroData[0] = sd.gyro_x;
			imu_new.gyroData[1] = sd.gyro_y;
			imu_new.gyroData[2] = sd.gyro_z;

			imu_new.accData[0] = sd.acc_x;
			imu_new.accData[1] = sd.acc_y;
			imu_new.accData[2] = sd.acc_z;

			imu_new.magData[0] = sd.mag_x;
			imu_new.magData[1] = sd.mag_y;
			imu_new.magData[2] = sd.mag_z;

			IMUTopic.publish(imu_new);


			sd.pitchAccMag = pitchAccMag;
			sd.rollAccMag = rollAccMag;
			sd.headingAccMag = headingAccMag;
			sd.pitchGyro = pitchGyro;
			sd.rollGyro = rollGyro;
			sd.headingGyro = headingGyro;
			sd.pitch = pitch;
			sd.heading = heading;
			sd.roll = roll;

			suspendUntilNextBeat();
		}
	}
};
SignalProc SignalProc("SignalProcessing");



