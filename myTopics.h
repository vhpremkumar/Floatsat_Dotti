/*
 * topic.h
 *
 *  Created on: 23.03.2015
 *      Author: Gageik
 */
#include "myStructs.h"

#ifndef TOPIC_H_
#define TOPIC_H_

extern Topic<IMU_Data> IMUTopic;
extern Topic<Kalman_Q_R> KalmanQRTopic;
extern Topic<Quaternion> QuatTopic;
extern Topic<Winkel> RPYTopic;
extern Topic<TM> TMTopic;
extern Topic<double> TempTopic;
extern Topic<double> DistanceTopic;
extern Topic<double> SolarTopic;
extern Topic<Battery> BatteryTopic;
extern Topic<int> MotorTopic;
extern Topic<double> RPMSteadyTopic;
extern Topic<double> ErrorRPMTopic;
extern Topic<double> RPMTopic;
extern Topic<double> RPMTargetTopic;
extern Topic<double> SpeedTargetTopic;
extern Topic<double> ControllerTargetTopic;
extern Topic<PID> ControllerPIDTopic;
extern Topic<PID> RPMControllerPIDTopic;
extern Topic<bool> StartOrbitTopic;
extern Topic<bool> CalibrationTopic;
extern Topic<bool> ThermalKnifeTopic;
extern Topic<int> ModeTopic;
extern Topic<Controllerstatus> ControllerstatusTopic;
extern Topic<double> OrbitalPeriodTopic;


extern CommBuffer<IMU_Data> IMUBuffer;
extern CommBuffer<Kalman_Q_R> KalmanQRBuffer;
extern CommBuffer<Quaternion> QuatBuffer;
extern CommBuffer<Winkel> RPYTopicBuffer;
extern CommBuffer<TM> TMTopicBuffer;
extern CommBuffer<double> TempBuffer;
extern CommBuffer<double> DistanceBuffer;
extern CommBuffer<Battery> BatteryBuffer;
extern CommBuffer<double> SolarBuffer;
extern CommBuffer<int> MotorBuffer;
extern CommBuffer<double> RPMSteadyBuffer;
extern CommBuffer<double> ErrorRPMBuffer;
extern CommBuffer<double> RPMBuffer;
extern CommBuffer<double> RPMTargetBuffer;
extern CommBuffer<double> SpeedTargetBuffer;
extern CommBuffer<double> ControllerTargetBuffer;
extern CommBuffer<PID> ControllerPIDBuffer;
extern CommBuffer<PID> RPMControllerPIDBuffer;
extern CommBuffer<bool> StartOrbitBuffer;
extern CommBuffer<bool> CalibrationBuffer;
extern CommBuffer<bool> ThermalKnifeBuffer;
extern CommBuffer<int> ModeBuffer;
extern CommBuffer<Controllerstatus> ControllerstatusBuffer;
extern CommBuffer<double> OrbitalPeriodBuffer;


#endif /* TOPIC_H_ */
