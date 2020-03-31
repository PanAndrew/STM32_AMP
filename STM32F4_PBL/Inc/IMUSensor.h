/*
 * IMUSensor.h
 *
 *  Created on: May 26, 2019
 *      Author: piotrek
 */

#ifndef IMUSENSOR_H_
#define IMUSENSOR_H_

#include "DataBuffer.h"
#include "IMUData.h"
#include "xNucleoShieldSensorsV3.h"
//#include "xNucleoShieldSensorsV1.h"
#include "main.h"

#define GYRO_CNT_REG_1 0x20
#define GYRO_TURN_ON 0x0F
#define GYRO_DATA 0x28
#define GYRO_MULTICONT_READ 0xC0
#define GYRO_DATA_SIZE 6

#define ACC_SENS_ADDR 0x32
#define ACC_DATA 0x28
#define ACC_MULTIBYTE_READ 0x80
#define ACC_CTRL_REG1 0x20
#define ACC_CTRL_TURN_ON 0x07
#define ACC_CTRL_200HZ 0x60
#define ACC_CTRL_REG1_200HZ (ACC_CTRL_TURN_ON | ACC_CTRL_200HZ)
#define ACC_CTRL_REG4 0x23
#define ACC_CTRL_HR 0x08
#define ACC_CTRL_2G 0x00
#define ACC_CTRL_REG4_2G_HR (ACC_CTRL_2G | ACC_CTRL_HR)
#define ACC_DATA_SIZE 6

#define MAG_SENS_ADDR 0x3C
#define MAG_DATA 0x03
#define MAG_CRA_REG 0x00
#define MAG_CRA_REG_30HZ 0x14
#define MAG_CRB_REG 0x01
#define MAG_CRB_REG_G_RANGE 0x40
#define MAG_MR_REG 0x02
#define MAG_MR_REG_CONT 0x00
#define MAG_DATA_SIZE 6


class IMUSensor {

	I2C_HandleTypeDef *i2cHandle;

	DataBuffer<IMUData> accBuffer;
	DataBuffer<IMUData> gyroBuffer;
	DataBuffer<IMUData> magBuffer;

	uint8_t rawData[6];
	bool readingInProgress = 0;
	uint32_t dataTimeStamp = 0;

	void pullAccData(I2C_HandleTypeDef *hi2c, uint8_t accDeviceAddr, uint8_t registerAddr, uint8_t size);
	void pullGyroData(I2C_HandleTypeDef *hi2c, uint8_t gyroDeviceAddr, uint8_t gyroDataAddr, uint8_t size);
	void pullGyroData(SPI_HandleTypeDef *hspi, uint8_t gyroDataAddr, uint8_t size);
	void pullMagData(I2C_HandleTypeDef *hi2c, uint8_t magDeviceAddr, uint8_t registerAddr, uint8_t size);

	void addToBuffer(DataBuffer<IMUData> &buffer, uint32_t timeStamp, uint8_t size);

	void writeSPI(SPI_HandleTypeDef *hspi, uint8_t *sensorRegAddr, uint8_t *regValue);
	void writeI2C(I2C_HandleTypeDef *hi2c, uint8_t sensorAddr , uint8_t *sensorRegAddr, uint8_t *regValue);

	void configureAcc(I2C_HandleTypeDef *hi2c, uint8_t accRegAddr, uint8_t regValue);
	void configureGyro(SPI_HandleTypeDef *hspi, uint8_t gyroRegAddr, uint8_t regValue);
	void configureGyro(I2C_HandleTypeDef *hi2c, uint8_t gyroRegAddr, uint8_t regValue);
	void configureMag(I2C_HandleTypeDef *hi2c, uint8_t accRegAddr, uint8_t regValue);

	uint16_t getBufferDataInArray(DataBuffer<IMUData> &buffer, uint8_t *dataBuffer);

public:
	IMUSensor(uint8_t buffersSize);
	virtual ~IMUSensor();

	void initializeI2C_Sensors(I2C_HandleTypeDef *hi2c);
	void initializeSPI_Sensors(SPI_HandleTypeDef *hspi);

	void pullDataFromSensorsI2C(I2C_HandleTypeDef *hi2c);
	void pullDataFromSensorsSPI(SPI_HandleTypeDef *hspi);

	uint16_t getAccData(uint8_t *dataBuffer);
	uint16_t getGyroData(uint8_t *dataBuffer);
	uint16_t getMagData(uint8_t *dataBuffer);

	void accRemoteConfiguration(uint8_t *dataBuffer);
	void gyroRemoteConfiguration(uint8_t *dataBuffer);
	void magRemoteConfiguration(uint8_t *dataBuffer);
};

#endif /* IMUSENSOR_H_ */
