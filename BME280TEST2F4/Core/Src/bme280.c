/*
 * bme280.c
 *
 *  Created on: Dec 6, 2022
 *      Author: erdog
 */

#include "BME280.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>




signed long temperature_raw;
signed long pressure_raw;

unsigned short dig_T1;
unsigned short dig_P1;
unsigned short dig_H1;
unsigned short dig_H3;
signed char dig_H6 ;
signed short dig_H5;
signed short dig_H4;
signed short dig_H2;
signed short dig_T2;
signed short dig_T3;
signed short dig_P2;
signed short dig_P3;
signed short dig_P4;
signed short dig_P5;
signed short dig_P6;
signed short dig_P7;
signed short dig_P8;
signed short dig_P9;
 //extern I2C_HandleTypeDef hi2c2;


/*uint8_t I2CRegisterRead(uint8_t dataAddr){
	uint8_t rxBuff;
	HAL_I2C_Mem_Read(&hi2c2, bme280deviceadd, dataAddr, 1, &rxBuff, 1, 1000);
	return rxBuff;
}
void I2CRegisterWrite(uint8_t dataAddr, uint8_t data){
	HAL_I2C_Mem_Write(&hi2c2, bme280deviceadd, dataAddr, 1, &data, 1, 1000);
}
void I2CRegisterWrite(uint8_t dataAddr, uint8_t data){
	HAL_I2C_Mem_Write(&hi2c2,bme280deviceadd , dataAddr, 1, &data, 1, 1000);
}
*/

float temperature;
float pressure;
float altitude;
float humanity;
