/*
 * Currentsensor.cpp
 *
 *  Created on: 03.02.2018
 *      Author: I8FL-PC01-G01
 */



#include "rodos.h"
#include "hal.h"
#include "math.h"
#include "misc.h"
#include "myStructs.h"
#include "myTopics.h"


HAL_GPIO HBRIDGE_D_INA(GPIO_076); /* declare HAL_GPIO for GPIO_036 = PC4 (HBRIDGE-A INA pin) */
HAL_GPIO HBRIDGE_D_INB(GPIO_079); /* declare HAL_GPIO for GPIO_017 = PB1 (HBRIDGE-B INA pin) */
HAL_GPIO HBRIDGE_EN2(GPIO_066); /* declare HAL_GPIO for GPIO_066 = PE2 (HBRIDGE Power Enable pin) */
HAL_PWM BridgeD(PWM_IDX15); /* declare HAL_PWM for PWM_IDX12 = TIM4-CH1 (HBRIDGE-A), please refer to hal_pwm.h for correct PWM mapping*/
#define SolarVoltageADC ADC_CH_002 // PA6 pin
HAL_ADC Solar(ADC_IDX2); /* declare HAL_ADC for ADC_IDX1, please refer to hal_adc.h for correct ADC mapping */
#define SolarCurrentADC ADC_CH_007 // PA7 pin
HAL_ADC Solar2(ADC_IDX3); /* declare HAL_ADC for ADC_IDX1, please refer to hal_adc.h for correct ADC mapping */


#define ADDRESS 0x40 /*Slave address*/
uint8_t ShuntVoltage[1] = {0x01};
uint8_t BusVoltage[1] = {0x02};
HAL_I2C CSensor(I2C_IDX1);
double status;
double full;
double empty;
Battery batterystatus;
bool knife;

class CurrentSensor: public Thread {

public:

CurrentSensor(const char* name) : Thread(name) {
	}

	void init() {
		CSensor.init(400000);
		BridgeD.init(5000, 1000); /* initialization of the HAL object should be called one time only in the project*/
		HBRIDGE_EN2.init(true, 1, 1); /* initialization of the HAL object should be called one time only in the project*/
		HBRIDGE_D_INA.init(true, 1, 0); /* initialization of the HAL object should be called one time only in the project*/
		HBRIDGE_D_INB.init(true, 1, 1); /* initialization of the HAL object should be called one time only in the project*/
		BridgeD.write(0);
		Solar.config(ADC_PARAMETER_RESOLUTION, 12);
		Solar.init(SolarVoltageADC); /* initialization of the HAL object should be called one time only in the project*/
		Solar2.config(ADC_PARAMETER_RESOLUTION, 12);
		Solar2.init(SolarCurrentADC);
		full = 13.15;
		empty = 0.9 * full;
	}

	void run() {
		knife = false;
		ThermalKnifeTopic.publish(knife);
		this->setPeriodicBeat(0, SENSE_SAMPLE_TIME*MILLISECONDS);
		while (1) {

			int32_t SolarVoltage = Solar.read(SolarVoltageADC);			
			int32_t SolarCurrent = Solar.read(SolarCurrentADC);			
			float SolarVoltagenorm = (SolarVoltage*9.9)/4096;		
			float SolarCurrentnorm = (((SolarCurrent*3.0)/4096)/1.5)*10;		
			float Power = (SolarVoltagenorm*SolarCurrentnorm);			
			batterystatus.SolarP = Power;

			uint8_t con[1] = {0};
			con[0] = 0x01;
			CSensor.write(ADDRESS, con, 1);
			uint8_t SVoltagedecimal[2];
			CSensor.writeRead(ADDRESS,ShuntVoltage, 1, SVoltagedecimal, 2);
			uint16_t SVoltage={0x0000 | SVoltagedecimal[0]<<8};
			SVoltage=SVoltage+SVoltagedecimal[1];
			float  Current=(SVoltage*0.00001)/0.002-0.17;
			batterystatus.I = Current;
			con[0] = 0x02;
			CSensor.write(ADDRESS, con, 1);
			uint8_t Voltagedecimal[2];
			CSensor.writeRead(ADDRESS,BusVoltage, 1, Voltagedecimal, 2);
			uint16_t BVoltage={0x0000 | Voltagedecimal[0]<<8};
			BVoltage=BVoltage+Voltagedecimal[1];
			BVoltage={0x0000 | BVoltage>>3};
			float result=BVoltage*0.004;		
			batterystatus.U = result;			
			if(result > full){
				status = 1;
			}
			else{
				status = (result-empty)/(full-empty);
				if(status < 0){
					status = 0;
				}
			}
			batterystatus.C = status*100;
			ThermalKnifeBuffer.get(knife);
			if(knife)
				{
				uint16_t duty=(7000/result);
				BridgeD.write(duty); /* Set the Duty Cycle to 25% */
				}
			else
				{
				BridgeD.write(0);
				}
			BatteryTopic.publish(batterystatus);			
			suspendUntilNextBeat();
		}
	}
};
CurrentSensor CurrentSensor("CurrentSensor");

