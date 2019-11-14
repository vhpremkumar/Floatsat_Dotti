/*
 * Telecommands.cpp
 *
 *  Created on: 18.12.2017
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
#include <vector>
#include <string>
#include <cstdlib>
#include "myStructs.h"

using std::vector;
using std::string;
using std::atoi;
using std::atof;


#define BUTTON GPIO_058
#define ADDR 0x39 /*Slave address*/

#define USART_BUFFER_SIZE				100

double data;
uint8_t dir;
TM readyglobal;
Motorspeed mspeed;
bool set;

//HAL_UART BT2UART(UART_IDX2);

namespace RODOS
{
	extern HAL_UART uart_stdout;
}
#define BT2UART uart_stdout

struct CmdStruct
{
	string cmdID;
	vector<string> arguments;
};

class Telecommands: public Thread {

private:
	TM tm;
	Controllerstatus controllerstatus;
	int mode = 0;
	int data1 = 0;
	int data2= 0;
	int data3= 0;
	PID pidvalues;
	Kalman_Q_R qr;

	vector<string> receiveMsgs(char *buffer, int size)
	{
		vector<string> out;

		int countBegin = -1;
		int countEnd = -1;
		PRINTF("JJReceived message\n");

		for (int i = 0; i<size; i++)
		{
			char c = buffer[i];
			if (c == '$' && countBegin == countEnd)
			{
				out.push_back("");
				countBegin++;
			}
			else if (countBegin != -1)
			{
				if (c == '#') {
					countEnd++;
				}
				else if (countBegin - countEnd == 1)
					out[countBegin] += c;
			}
		}
		return out;
	}

	CmdStruct getCmd(string &msg)
	{
		bool firstArg = true;
		CmdStruct cmd;
		string tmp = "";
		for (int i = 0; i < msg.length(); i++)
		{
			if (msg[i] == ',')
			{
				if (firstArg)
				{
					cmd.cmdID = tmp;
					firstArg = false;
				}
				else
				{
					cmd.arguments.push_back(tmp);
				}
				tmp = "";
			}
			else
				tmp += msg[i];
		}
		return cmd;

	}

public:

	Telecommands(const char* name) : Thread(name) {
	}

	void init() {
		//BT2UART.init(921600);
		//BT2UART.config(UART_PARAMETER_ENABLE_DMA, 1);
		readyglobal.sendDistance = false;
		readyglobal.sendLight = false;
		readyglobal.sendRPY = false;
		readyglobal.sendIMU = false;
		readyglobal.sendBattery = false;
		controllerstatus.RPM=0;
		controllerstatus.position=0;
		controllerstatus.speed=0;

	}

	void run() {

		//EncoderInit();
		this->setPeriodicBeat(0, 40*MILLISECONDS);

				uint64_t uart_read_size;
				char usart_buffer[USART_BUFFER_SIZE + 1];

				while (1) {

					uart_read_size = BT2UART.read(usart_buffer, USART_BUFFER_SIZE);
					usart_buffer[uart_read_size] = '\0';
					if (uart_read_size > 0)
					{
						#ifdef TC_ECHO
							uart_stdout.write(usart_buffer, uart_read_size);
						#endif

						vector<string> msgs = receiveMsgs(usart_buffer, uart_read_size);

						for (int i = 0; i < msgs.size(); i++)
						{
							if (msgs[i].length() > 0)
							{
								PRINTF("JJThe message is %s \n",msgs[i].c_str());
								CmdStruct cmd = getCmd(msgs[i]);

								if (cmd.cmdID == "B")
								{
									if (cmd.arguments.size() == 1) {
										data = (uint8_t)atoi(cmd.arguments[0].c_str());
										if(data==0)
											{
											readyglobal.sendRPY=false;
											}
										else
										    {
											readyglobal.sendRPY=true;
										    }
									}
								}
								else if (cmd.cmdID == "C")
								{
									if (cmd.arguments.size() == 3) {
										data1 = (uint8_t)atoi(cmd.arguments[0].c_str());
										data2 = (uint8_t)atoi(cmd.arguments[1].c_str());
										data3 = (uint8_t)atoi(cmd.arguments[2].c_str());
										if(data1==0)
											{
											readyglobal.sendDistance=false;
											}
										else
										    {
											readyglobal.sendDistance=true;
										    }
										if(data2==0)
											{
											readyglobal.sendLight=false;
											}
										else
										    {
											readyglobal.sendLight=true;
										    }
										if(data3==0)
											{
											readyglobal.sendBattery=false;
											}
										else
										    {
											readyglobal.sendBattery=true;
										    }
									}
								}

								else if (cmd.cmdID == "A")
								{
									if (cmd.arguments.size() == 1) {
										data = strtod(cmd.arguments[0].c_str(),0);
										if(data<=-180 || data > 180)
											{
											controllerstatus.position = 0;
											controllerstatus.RPM = 0;
											controllerstatus.speed = 0;
											data = 0;
											RPMTargetTopic.publish(data);
											data1 = 0;
											MotorTopic.publish(data1);
											}
										else
										    {

											controllerstatus.position = 1;
											controllerstatus.RPM = 1;
											controllerstatus.speed = 1;
											ControllerTargetTopic.publish(data);
											data1 = 1;
											MotorTopic.publish(data1);
										    }
									}
									ControllerstatusTopic.publish(controllerstatus);
								}
								else if (cmd.cmdID == "E")
								{
									if (cmd.arguments.size() == 1) {
										data = strtod(cmd.arguments[0].c_str(),0);
										RPMTargetTopic.publish(data);
										controllerstatus.position = 0;
										controllerstatus.RPM = 1;
										controllerstatus.speed = 0;
										ControllerstatusTopic.publish(controllerstatus);
										data1 = 1;
										MotorTopic.publish(data1);
									}
								}
								else if (cmd.cmdID == "P")
								{
									if (cmd.arguments.size() == 3) {
										pidvalues.P = strtod(cmd.arguments[0].c_str(),0);
										pidvalues.I = strtod(cmd.arguments[1].c_str(),0);
										pidvalues.D = strtod(cmd.arguments[2].c_str(),0);
										PRINTF("JJPnew: %f\n", pidvalues.P);
										PRINTF("JJInew: %f\n", pidvalues.I);
										PRINTF("JJDnew: %f\n", pidvalues.D);
										ControllerPIDTopic.publish(pidvalues);
									}
								}
								else if (cmd.cmdID == "p")
								{
									if (cmd.arguments.size() == 3) {
										pidvalues.P = strtod(cmd.arguments[0].c_str(),0);
										pidvalues.I = strtod(cmd.arguments[1].c_str(),0);
										pidvalues.D = strtod(cmd.arguments[2].c_str(),0);
										PRINTF("JJRPM Pnew: %f\n", pidvalues.P);
										PRINTF("JJRPM Inew: %f\n", pidvalues.I);
										PRINTF("JJRPM Dnew: %f\n", pidvalues.D);
										RPMControllerPIDTopic.publish(pidvalues);
									}
								}
								else if (cmd.cmdID == "F")
								{
									if (cmd.arguments.size() == 4) {
										double a = strtod(cmd.arguments[0].c_str(),0);

										double b = strtod(cmd.arguments[1].c_str(),0);
										double c = strtod(cmd.arguments[2].c_str(),0);
										double d = strtod(cmd.arguments[3].c_str(),0);
										qr.Q = Matrix2D(pow(10.0,a),0,0,pow(10.0,b));
										qr.R = Matrix2D(pow(10.0,c),0,0,pow(10.0,d));
										PRINTF("JJa: %f\n",qr.Q.a11);
										PRINTF("JJb: %f\n",qr.Q.a22);
										PRINTF("JJc: %f\n",qr.R.a11);
										PRINTF("JJd: %f\n",qr.R.a22);
										KalmanQRTopic.publish(qr);
									}
								}
								else if (cmd.cmdID == "S")
								{
									if (cmd.arguments.size() == 1) {
										double a = strtod(cmd.arguments[0].c_str(),0);
										PRINTF("JJSetting rotationspeed to %f deg per second\n\n",a);
										SpeedTargetTopic.publish(a);
										data1 = 1;
										MotorTopic.publish(data1);
										controllerstatus.position = 0;
										controllerstatus.RPM = 1;
										controllerstatus.speed = 1;
										ControllerstatusTopic.publish(controllerstatus);
									}
								}
								else if (cmd.cmdID == "O")
								{
									if (cmd.arguments.size() == 1) {
										data = (uint8_t)atoi(cmd.arguments[0].c_str());
										if (data == 1){
											bool start = true;
											StartOrbitTopic.publish(start);
										}
									}
								}
								else if (cmd.cmdID == "R")
								{
									if (cmd.arguments.size() == 1) {
										data = 0;
										controllerstatus.position = 0;
										controllerstatus.RPM = 0;
										controllerstatus.speed = 0;
										ControllerstatusTopic.publish(controllerstatus);
										RPMTargetTopic.publish(data);
										ControllerTargetTopic.publish(data);
										data1 = 0;
										MotorTopic.publish(data1);
									}
								}
								else if (cmd.cmdID == "Y")
								{
									if (cmd.arguments.size() == 1) {
										set = true;
										CalibrationTopic.publish(set);
									}
								}
								else if (cmd.cmdID == "T")
								{
									if (cmd.arguments.size() == 1) {
										data = (uint8_t)atoi(cmd.arguments[0].c_str());
										if (data == 1){
											set = true;
											PRINTF("JJThermalknife Command on received");
										}
										else{
											set = false;
											PRINTF("JJThermalknife Command off received");
										}
										ThermalKnifeTopic.publish(set);
									}
								}
								else if (cmd.cmdID == "M")
								{
									if (cmd.arguments.size() == 1) {
										mode = (uint8_t)atoi(cmd.arguments[0].c_str());
										ModeTopic.publish(mode);
										PRINTF("JJMode set to %d\n", mode);
									}
								}
								TMTopic.publish(readyglobal);
							}
						}
					}

				suspendUntilNextBeat();
	}
};
Telecommands Telecommands("Telecommands");


