/*
 * filter.cpp
 *
 *  Created on: 28.11.2017
 *      Author: I8FL-PC01-G01
 */

#include "Filter.h"
#include "myMath.h"
#include "myTopics.h"
#include <math.h>
#include "rodos.h"
#include "Vec2D.h"
#include "Matrix2D.h"


class Filter_KF: public Thread {

	KalmanFilter kf_roll;
	KalmanFilter kf_pitch;
	KalmanFilter kf_yaw;

	bool ready;

public:
	Filter_KF(const char* name, const long prio) :
			Thread(name, prio) {
		ready = true;

	}

	void run(){

		PRINTF("JJ\n\n*** KF initialized ***\n");
		this->setPeriodicBeat(1*SECONDS, 10*MILLISECONDS);
		
		while(1){
			
			if(ready)
			{
				
				IMU_Data imu_new;
				IMUBuffer.get(imu_new);

				double ax = imu_new.accData[0];
				double ay = imu_new.accData[1];
				double az = imu_new.accData[2];

				double mx = imu_new.magData[0];
				double my = imu_new.magData[1];
				double mz = imu_new.magData[2];

				double pitchAcc = atan2(-ax, sqrt(ay*ay+az*az));
				double rollAcc = atan2(ay, sqrt(ax*ax+az*az));


				kf_roll.computeNewValues(Vec2D(rollAcc, imu_new.gyroData[0] * M_PI/180));
				kf_pitch.computeNewValues(Vec2D(pitchAcc, imu_new.gyroData[1] * M_PI/180));

				Winkel angles;
				angles.roll = kf_roll.xk.x * 180/M_PI;
				angles.pitch = kf_pitch.xk.x * 180/M_PI;

				double mxh = mx*cos(GRAD2RAD(angles.pitch)) + mz*sin(GRAD2RAD(angles.pitch));
				double myh = mx*sin(GRAD2RAD(angles.roll))*sin(GRAD2RAD(angles.pitch)) + my*cos(GRAD2RAD(angles.roll)) - mz*sin(GRAD2RAD(angles.roll))*cos(GRAD2RAD(angles.pitch));

				double headingAccMag = atan2(myh, mxh);

				kf_yaw.computeNewValues(Vec2D(headingAccMag, imu_new.gyroData[2] * M_PI/180));

				angles.yaw = kf_yaw.xk.x * 180/M_PI;
				RPYTopic.publish(angles);
			}
			suspendUntilNextBeat();
		}

	}

};

KalmanFilter::KalmanFilter() {
	A = Matrix2D(1, 0.01, 0, 1);
	C = Matrix2D(1, 0, 0, 1);
	//CHANGE FILTER VALUES HERE
	qr.Q = Matrix2D(1, 0, 0, 1);
	qr.R = Matrix2D(10, 0, 0, 1);
	Pk = Matrix2D(10, 0, 0, 10);
	I = Matrix2D(1, 0, 0, 1);
	xk = Vec2D(0, 0);
	KalmanQRTopic.publish(qr);
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::computeNewValues(Vec2D yk)
{
	KalmanQRBuffer.get(qr);

	Vec2D xks = A * xk;
	if(((sum(yk.x) + sum(xks.x)) > M_PI) && ((sign(yk.x) * sign(xks.x)) < 0)){		
		if(sum(yk.x) > sum(xks.x)){
			yk.x -= sign(yk.x)*(2*M_PI);
		}
		else{
			xks.x -= sign(xks.x)*(2*M_PI);
		}
		
	}
	Matrix2D Pks = A*Pk*A.transposed() + qr.Q;
	Kk = Pks * C.transposed() * (C * Pks * C.transposed() + qr.R).inverted();
	xk = xks + Kk * (yk - C * xks);	
	Pk = (I - Kk * C) * Pks;
}

Filter_KF 		 	kf("Kalman Filter", 100);
