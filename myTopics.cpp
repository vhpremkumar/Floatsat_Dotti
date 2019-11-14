/*
 * myTopics.cpp
 *
 *  Created on: 23.03.2015
 *      Author: Gageik
 */

#include "rodos.h"
#include "myStructs.h"



Topic<IMU_Data> IMUTopic(-1, "IMU");
Topic<Kalman_Q_R> KalmanQRTopic(-1, "Kalman Q R");
Topic<Quaternion> QuatTopic(-1, "Quaternion");
Topic<Winkel> RPYTopic(-1,"Filter Angels");
Topic<TM> TMTopic(-1, "Telemetry Ready");
Topic<double> TempTopic(-1, "IMU Temperature");
Topic<int> MotorTopic(-1, "Motormode set by telecommand");
Topic<double> RPMSteadyTopic(-1, "Steady state RPM");
Topic<double> ErrorRPMTopic(-1, " RPM Error determined by the controller");
Topic<double> RPMTopic(-1, "MotorRPM determined by Encoder");
Topic<double> RPMTargetTopic(-1, "RPMTarget");
Topic<double> SpeedTargetTopic(-1, "Speed Target");
Topic<double> ControllerTargetTopic(-1, "desired yaw");
Topic<PID> ControllerPIDTopic(-1, "sets PID values of controller");
Topic<PID> RPMControllerPIDTopic(-1, "sets PID values of rpmcontroller");
Topic<double> DistanceTopic(-1, "Distance Sensor Value");
Topic<Battery> BatteryTopic(-1, "Current Sensor Value");
Topic<double> SolarTopic(-1, "Light Sensor Value");
Topic<bool> StartOrbitTopic(-1, "Starts Calculating the Orbital Period");
Topic <bool> CalibrationTopic(-1, "Starts Calibrating");
Topic <bool> ThermalKnifeTopic(-1, "Turns on/off Thermal Knife");
Topic <int> ModeTopic (-1, "Mode the satellite is currently in");
Topic <Controllerstatus> ControllerstatusTopic (-1, "Turns on and off the controllers");
Topic <double> OrbitalPeriodTopic (-1, "Orbital Period of the Object");


CommBuffer<IMU_Data> IMUBuffer;
Subscriber IMUSub(IMUTopic, IMUBuffer);

CommBuffer<Kalman_Q_R> KalmanQRBuffer;
Subscriber KalmanQRSub(KalmanQRTopic, KalmanQRBuffer);

CommBuffer<Quaternion> QuatBuffer;
Subscriber QuatSub(QuatTopic, QuatBuffer);

CommBuffer<Winkel> RPYTopicBuffer;
Subscriber RPYSub(RPYTopic, RPYTopicBuffer);

CommBuffer<TM> TMTopicBuffer;
Subscriber TMSub(TMTopic, TMTopicBuffer);

CommBuffer<double> TempBuffer;
Subscriber TempSub(TempTopic, TempBuffer);

CommBuffer<double> DistanceBuffer;
Subscriber DistanceSub(DistanceTopic, DistanceBuffer);

CommBuffer<Battery> BatteryBuffer;
Subscriber BatterySub(BatteryTopic, BatteryBuffer);

CommBuffer<double> SolarBuffer;
Subscriber SolarSub(SolarTopic, SolarBuffer);

CommBuffer<int> MotorBuffer;
Subscriber MotorSub(MotorTopic, MotorBuffer);

CommBuffer<double> RPMSteadyBuffer;
Subscriber RPMSteadySub(RPMSteadyTopic, RPMSteadyBuffer);

CommBuffer<double> ErrorRPMBuffer;
Subscriber ErrorRPMSub(ErrorRPMTopic, ErrorRPMBuffer);

CommBuffer<double> RPMBuffer;
Subscriber RPMSub(RPMTopic, RPMBuffer);

CommBuffer<double> RPMTargetBuffer;
Subscriber RPMTargetSub(RPMTargetTopic, RPMTargetBuffer);

CommBuffer<double> SpeedTargetBuffer;
Subscriber SpeedTargetSub(SpeedTargetTopic, SpeedTargetBuffer);

CommBuffer<double> ControllerTargetBuffer;
Subscriber ControllerTargetSub(ControllerTargetTopic, ControllerTargetBuffer);

CommBuffer<PID> ControllerPIDBuffer;
Subscriber ControllerPIDSub(ControllerPIDTopic, ControllerPIDBuffer);

CommBuffer<PID> RPMControllerPIDBuffer;
Subscriber RPMControllerPIDSub(RPMControllerPIDTopic, RPMControllerPIDBuffer);

CommBuffer<bool> StartOrbitBuffer;
Subscriber StartOrbitSub(StartOrbitTopic, StartOrbitBuffer);

CommBuffer<bool> CalibrationBuffer;
Subscriber CalibrationSub(CalibrationTopic, CalibrationBuffer);

CommBuffer<bool> ThermalKnifeBuffer;
Subscriber ThermalKnifeSub(ThermalKnifeTopic, ThermalKnifeBuffer);

CommBuffer<int> ModeBuffer;
Subscriber ModeSub(ModeTopic, ModeBuffer);

CommBuffer<Controllerstatus> ControllerstatusBuffer;
Subscriber ControllerstatusSub(ControllerstatusTopic, ControllerstatusBuffer);

CommBuffer<double> OrbitalPeriodBuffer;
Subscriber OrbitalPeriodSub(OrbitalPeriodTopic, OrbitalPeriodBuffer);
