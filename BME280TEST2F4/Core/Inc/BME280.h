/*
 * BME280.h
 *
 *  Created on: Dec 6, 2022
 *      Author: erdog
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "math.h"






#define bme280deviceadd 0x77<<1
#define hum_lsb  0xFE
#define hum_msb 0xFD
#define temp_xlsb 0xFC
#define temp_lsb 0xFB
#define temp_msb 0xFA
#define press_xlsb 0xF9
#define press_lsb 0xF8
#define press_msb 0xF7
//CONFİG//////////////////////
#define CONFIG 0xF5
//STATUSS/////////////////////
#define STATUS  0xF3
#define ctrl_hum 0xF2
#define reset 0xE0
#define bme_id 0xD0
#define calib00 0x88
#define calib25 0xA1


//ctrl meas
#define CTRL_MEAS 0xF4
#define STATUS_CONV 0x08
#define STATUS_COPY 0x01


// pressure oversampling
#define OVERSAMPLING_PRESSURE_1BIT 0x01 // x1
#define OVERSAMPLING_PRESSURE_2BIT 0x02 // x2
#define OVERSAMPLING_PRESSURE_3BIT 0x03 // x4
#define OVERSAMPLING_PRESSURE_4BIT 0x04 // x8
#define OVERSAMPLING_PRESSURE_5BIT 0x05 // x16 or 6,7,8
//tempereture over samping
#define OVERSAMPLING_TEMPERATURE_1BIT 0x01
#define OVERSAMPLING_TEMPERATURE_2BIT 0x02
#define OVERSAMPLING_TEMPERATURE_3BIT 0x03
#define OVERSAMPLING_TEMPERATURE_4BIT 0x04
#define OVERSAMPLING_TEMPERATURE_5BIT 0x05
//HUMADİTY OVERSAMPLİNG
#define OVERSAMPLING_HUMADITY_1BIT 0x01
#define OVERSAMPLING_HUMADITY_2BIT 0x02
#define OVERSAMPLING_HUMADITY_3BIT 0x03
#define OVERSAMPLING_HUMADITY_4BIT 0x04
#define OVERSAMPLING_HUMADITY_5BIT 0x05
//modları
#define SLEEP  0x00
#define FORCED 0x01 // or 0x02
#define NORMAL 0x03
//			Temperature Offset
#define CTRL_MEAS_T_OFFSET 0x05 //7 yi de dene
//			Pressure Offset
#define CTRL_MEAS_P_OFFEST 0x02 //4 ü dene
//			Humidty OFFSETT
#define CTRL_MEAS_H_OFFSETT 0x00
//			Power Mode Offset
#define CTRL_MEAS_M_OFFEST 0x00 //1 i dene
//Temperature oversampling
#define OVERSAMPLING_T_1BIT (OVERSAMPLING_TEMPERATURE_1BIT  << CTRL_MEAS_T_OFFSET)
#define OVERSAMPLING_T_2BIT (OVERSAMPLING_TEMPERATURE_2BIT << CTRL_MEAS_T_OFFSET)
#define OVERSAMPLING_T_3BIT (OVERSAMPLING_TEMPERATURE_3BIT << CTRL_MEAS_T_OFFSET)
#define OVERSAMPLING_T_4BIT (OVERSAMPLING_TEMPERATURE_4BIT << CTRL_MEAS_T_OFFSET)
#define OVERSAMPLING_T_5BIT (OVERSAMPLING_TEMPERATURE_5BIT << CTRL_MEAS_T_OFFSET)
//PRESSURE OVERSAMPLİNG
#define OVERSAMPLING_P_1BIT (OVERSAMPLING_PRESSURE_1BIT    << CTRL_MEAS_P_OFFEST)
#define OVERSAMPLING_P_2BIT (OVERSAMPLING_PRESSURE_2BIT	   <<CTRL_MEAS_P_OFFEST )
#define OVERSAMPLING_P_3BIT (OVERSAMPLING_PRESSURE_3BIT    <<CTRL_MEAS_P_OFFEST )
#define OVERSAMPLING_P_4BIT (OVERSAMPLING_PRESSURE_4BIT    <<CTRL_MEAS_P_OFFEST )
#define OVERSAMPLING_P_5BIT (OVERSAMPLING_PRESSURE_5BIT    <<CTRL_MEAS_P_OFFEST )
//HUMİDTY OVERSAMPLİNG
#define OVERSAMPLING_H_1BIT (OVERSAMPLING_HUMADITY_1BIT   << CTRL_MEAS_H_OFFEST)
#define OVERSAMPLING_H_2BIT (OVERSAMPLING_HUMADITY_2BIT	  <<CTRL_MEAS_H_OFFEST )
#define OVERSAMPLING_H_3BIT (OVERSAMPLING_HUMADITY_3BIT   <<CTRL_MEAS_H_OFFEST )
#define OVERSAMPLING_H_4BIT (OVERSAMPLING_HUMADITY_4BIT   <<CTRL_MEAS_H_OFFEST )
#define OVERSAMPLING_H_5BIT (OVERSAMPLING_HUMADITY_5BIT   <<CTRL_MEAS_H_OFFEST )
//				Power Mode Selection
#define SLEEP_MODE  (SLEEP  << CTRL_MEAS_M_OFFEST);
#define FORCED_MODE (FORCED << CTRL_MEAS_M_OFFEST);
#define NORMAL_MODE (NORMAL << CTRL_MEAS_M_OFFEST);


//		T_SB_OFFSET
#define T_SB_OFFSET 0x05
//		FILTER_OFFSET
#define FILTER_OFFSET 0x02

//			T_SB SETTING (ms)
#define T_SB_0_5  (0x00 << T_SB_OFFSET)
#define T_SB_62_5 (0x01 << T_SB_OFFSET)
#define T_SB_125  (0x02 << T_SB_OFFSET)
#define T_SB_250  (0x03 << T_SB_OFFSET)
#define T_SB_500  (0x04 << T_SB_OFFSET)
#define T_SB_1000 (0x05 << T_SB_OFFSET)
#define T_SB_10   (0x06 << T_SB_OFFSET)
#define T_SB_20   (0x07 << T_SB_OFFSET)

//			FILTER
#define FILTER_2  (0x02 << FILTER_OFFSET)
#define FILTER_5  (0x04 << FILTER_OFFSET)
#define FILTER_11 (0x08 << FILTER_OFFSET)
#define FILTER_22 (0x10 << FILTER_OFFSET)


// PRESSURE CALIBRATION DATA
#define CALIB_DATA_P_ADDR 0xA1
#define CALIB_DATA_P_SIZE 24
//  HUMADİTY CALIBRATİON DATA
#define  CALIB_DATA_H_ADDR 0xF0
#define  CALIB_DATA_H_SIZE 15
// RAW DATA READING
#define START_RAW_DATA_POINT 0xF7
#define RAW_DATA_LENGTH 6
//extern I2C_HandleTypeDef hi2c2;

uint8_t I2CRegisterRead(
		uint8_t dataAddr
		);

void I2CRegisterWrite(
		uint8_t dataAddr,
		uint8_t data
		);

//void calibrationOfBMP280(void);

void BME280Init(uint8_t ctrl_meas,uint8_t config);

//void BMP280Calculation(void);









#endif /* INC_BME280_H_ */
