/*
 * LightSensor.cpp
 *
 *  Created on: 18.12.2017
 *      Author: I8FL-PC01-G01
 */

#include "rodos.h"
#include <stdio.h>
#include "hal.h"
#include "myTopics.h"
#include "myStructs.h"

//#define Analog ADC_CH_001 // PA1 pin
#define ADDR 0x49 /*Slave address*/
uint8_t Channel0[1] = {0xAC};
uint8_t Channel1[1] = {0xAE};
HAL_I2C LSensor(I2C_IDX2);

//HAL_ADC InfraRed(ADC_IDX1);
TM ready2;
int countersun;
int countergoalsun;

class LightSensor: public Thread {

	uint8_t data1[2];
	uint16_t data3;
	uint8_t data2[2];
	uint16_t data4;
	double data5;

public:

LightSensor(const char* name) : Thread(name) {
	}

	void init() {
		countergoalsun = 100000;
		countersun = 0;
		LSensor.init(100000); /* initialization of the HAL object should be called one time only in the project*/
		//AD.init(true, 1, 1);
		// Select control register(0x00 | 0x80)
		// Power ON mode(0x03)
		uint8_t config[2] = {0};
		config[0] = 0x00 | 0x80;
		config[1] = 0x03;
		LSensor.write(ADDR, config, 2);
		// Select timing register(0x01 | 0x80)
		// Nominal integration time = 402ms(0x02)
		config[0] = 0x01 | 0x80;
		config[1] = 0x02;
		LSensor.write(ADDR, config, 2);
		uint8_t reg[1] = {0x0C | 0x80};
		LSensor.write(ADDR, reg, 1);


	}

	void run() {
		ready2.sendDistance = true;
		ready2.sendLight = true;
		this->setPeriodicBeat(0, SENSE_SAMPLE_TIME*MILLISECONDS);


		while (1) {
			countersun++;
			data1[0] = 0;
			data1[1] = 0;
			LSensor.writeRead(ADDR,Channel0, 1, data1, 2);
			data3={0x0000 | data1[1]<<8};
			data3=data3+data1[0];
			data2[0] = 0;
			data2[1] = 0;
			LSensor.writeRead(ADDR,Channel1, 1, data2, 2);		
			data4={0x0000 | data2[1]<<8};
			data4=data4+data2[0];		
			data5=data3+data4;
			if (countersun == countergoalsun){
				countersun = 0;
				PRINTF("JJTotal light sensor reading is = %f \r\n", data5);
			}
			SolarTopic.publish(data5);
			suspendUntilNextBeat();

		}
	}
};
LightSensor LightSensor("LightSensor");


