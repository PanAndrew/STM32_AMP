/*
 * IMUSensor.cpp
 *
 *  Created on: May 26, 2019
 *      Author: piotrek
 */

#include "IMUSensor.h"

IMUSensor::IMUSensor(uint8_t buffersSize):
	accBuffer(buffersSize), gyroBuffer(buffersSize), magBuffer(buffersSize) {
	// TODO Auto-generated constructor stub

}

IMUSensor::~IMUSensor() {
	// TODO Auto-generated destructor stub
}

void IMUSensor::initialize(SPI_HandleTypeDef *hspi, I2C_HandleTypeDef *hi2c)
{
	configureGyro(hspi, GYRO_CNT_REG_1, GYRO_TURN_ON);
	configureAcc(hi2c, ACC_CTRL_REG1, ACC_CTRL_REG1_200HZ);
	configureMag(hi2c, MAG_CRA_REG, MAG_CRA_REG_30HZ);
	configureMag(hi2c, MAG_MR_REG, MAG_MR_REG_CONT);
}

void IMUSensor::addToBuffer(DataBuffer<IMUData> &buffer, uint8_t size)
{
	IMUData measurement(rawData, HAL_GetTick(), size);
	buffer.put(measurement);
}

void IMUSensor::pullAccData(I2C_HandleTypeDef *hi2c, uint8_t accDeviceAddr, uint8_t registerAddr, uint8_t size)
{
	HAL_I2C_Mem_Read(hi2c, accDeviceAddr, registerAddr, size, rawData, 1, 1);

	addToBuffer(accBuffer, size);
}

void IMUSensor::pullGyroData(SPI_HandleTypeDef *hspi, uint8_t gyroDataAddr, uint8_t size)
{
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(hspi, &gyroDataAddr, 1, 1);
	  HAL_SPI_Receive(hspi, rawData, size, 1);
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);

	  for(uint8_t i = 0; i < 6; i+=2)
	  {
		  std::swap(rawData[i], rawData[i + 1]);
	  }

	  addToBuffer(gyroBuffer, size);
}

void IMUSensor::pullMagData(I2C_HandleTypeDef *hi2c, uint8_t magDeviceAddr, uint8_t registerAddr, uint8_t size)
{
	HAL_I2C_Mem_Read(hi2c, magDeviceAddr, registerAddr, size, rawData, 1, 1);

	std::swap(rawData[2], rawData[4]);
	std::swap(rawData[3], rawData[5]);
	addToBuffer(magBuffer, size);
}

void IMUSensor::pullDataFromSensors(SPI_HandleTypeDef *hspi, I2C_HandleTypeDef *hi2c)
{
	pullGyroData(hspi, GYRO_DATA | GYRO_MULTICONT_READ, GYRO_DATA_SIZE);
	pullAccData(hi2c, ACC_SENS_ADDR, ACC_DATA | ACC_MULTIBYTE_READ, ACC_DATA_SIZE);
	pullMagData(hi2c, MAG_SENS_ADDR, MAG_DATA, MAG_DATA_SIZE);
}

void IMUSensor::writeSPI(SPI_HandleTypeDef *hspi, uint8_t *sensorRegAddr, uint8_t *regValue)
{
	uint8_t commandToSend[2] = {*sensorRegAddr, *regValue};
	HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, commandToSend, 2, 1);
	HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);
}

void IMUSensor::writeI2C(I2C_HandleTypeDef *hi2c, uint8_t sensorAddr , uint8_t *sensorRegAddr, uint8_t *regValue)
{
	HAL_I2C_Mem_Write(hi2c, sensorAddr, *sensorRegAddr, 1, regValue, 1, 1);
}

void IMUSensor::configureAcc(I2C_HandleTypeDef *hi2c, uint8_t accRegAddr, uint8_t regValue)
{
	writeI2C(hi2c, ACC_SENS_ADDR, &accRegAddr, &regValue);
}

void IMUSensor::configureGyro(SPI_HandleTypeDef *hspi, uint8_t gyroRegAddr, uint8_t regValue)
{
	writeSPI(hspi, &gyroRegAddr, &regValue);
}

void IMUSensor::configureMag(I2C_HandleTypeDef *hi2c, uint8_t magRegAddr, uint8_t regValue)
{
	writeI2C(hi2c, MAG_SENS_ADDR, &magRegAddr, &regValue);
}

uint8_t IMUSensor::getAccData(uint8_t *dataBuffer)
{
	IMUData tempData;
	uint8_t *ptrToArrayData;
	uint8_t numberOfElements = accBuffer.size();

	for(uint16_t i = 0; i < accBuffer.size(); i++)
	{
		tempData = accBuffer.get();
		ptrToArrayData = tempData.getDataInArray();
		std::copy(ptrToArrayData, ptrToArrayData + tempData.getSize(), dataBuffer);

		//TO DO
		//dopisanie informacji do dataBuffer'a (dopisywanie w pętli całego buffora)
		//głowny dataManagement
		//swap danych y i z V
	}

	return numberOfElements;
}

uint8_t IMUSensor::getGyroData(uint8_t *dataBuffer)
{
	IMUData tempData;
	uint8_t *ptrToArrayData;
	uint8_t numberOfElements = gyroBuffer.size();

	for(uint16_t i = 0; i < gyroBuffer.size(); i++)
	{
		tempData = gyroBuffer.get();
		ptrToArrayData = tempData.getDataInArray();
		std::copy(ptrToArrayData, ptrToArrayData + tempData.getSize(), dataBuffer);
	}
	return numberOfElements;
}

uint8_t IMUSensor::getMagData(uint8_t *dataBuffer)
{
	IMUData tempData;
	uint8_t *ptrToArrayData;
	uint8_t numberOfElements = magBuffer.size();

	for(uint16_t i = 0; i < magBuffer.size(); i++)
	{
		tempData = magBuffer.get();
		ptrToArrayData = tempData.getDataInArray();
		std::copy(ptrToArrayData, ptrToArrayData + tempData.getSize(), dataBuffer);
	}

	return numberOfElements;
}
