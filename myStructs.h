/*
 * myStructs.h
 *
 *  Created on: 31.03.2015
 *      Author: Gageik
 */

#include "matlib.h"
#include "rodos.h"
#include "myMath.h"

#include "Vec2D.h"
#include "Matrix2D.h"

#ifndef MYSTRUCTS_H_
#define MYSTRUCTS_H_


#define RAD2GRAD(x)			((x)*180/M_PI)
#define GRAD2RAD(x)			((x)*M_PI/180)
#define SENSE_SAMPLE_TIME			10


typedef struct{
	unsigned char M1;
	unsigned char M2;
	unsigned char M3;
	unsigned char M4;
}Stellwerte;


typedef struct{
	double roll, pitch, yaw;
}Winkel;

typedef struct{						// Rohwerte
	double gyroData[3];
	double accData[3];
	double magData[3];
}IMU_Data;

typedef struct{
	Matrix2D K_roll;
	Matrix2D P_roll;
	Matrix2D K_pitch;
	Matrix2D P_pitch;
}Kalman_Data;


typedef struct{
	HAL_I2C * i2c;
	bool i2c_init;
	int	error_counter;
	uint8_t i2c_buffer[12];
	int32_t i2c_return;
}I2C_Data;

typedef struct{
	Matrix2D Q;
	Matrix2D R;
} Kalman_Q_R;

typedef struct{
	double P;
	double I;
	double D;
} PID;

typedef struct{
	double U;
	double I;
	double C;
	double SolarP;
} Battery;

typedef struct{
	int position;
	int speed;
	int RPM;
} Controllerstatus;

typedef struct{
	double P_part;
	double I_part;
	double D_part;
	double total;
} PID_ATC;

typedef struct{
	bool engine_on;
	bool engine_switch_mode_on;
	bool engine_manual_control;
	bool engine_sequence_hash;
	bool engine_sequence_rect;
	bool kalman_on;
} Engine_State;

typedef struct{
	bool sendDistance;
	bool sendRPY;
	bool sendLight;
	bool sendIMU;
	bool sendBattery;

} TM;

typedef struct{
	uint16_t speed;
	bool direction;
} Motorspeed;

#endif /* MYSTRUCTS_H_ */
