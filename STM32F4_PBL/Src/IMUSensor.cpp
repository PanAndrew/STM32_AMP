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

void IMUSensor::initializeI2C_Sensors(I2C_HandleTypeDef *hi2c)
{
	i2cHandle = hi2c;

#ifdef XNucleoV3
	configureAcc(hi2c, LSM6DSO_ACC_CTRL_REG1, LSM6DSO_ACC_CTRL_REG1_VALUE);
	configureGyro(hi2c, LSM6DSO_GYRO_CTRL_REG_2, LSM6DSO_GYRO_CTRL_REG_2_VALUE);
	configureMag(hi2c, LIS2MDL_MAG_CFG_REG_A, LIS2MDL_MAG_CFG_REG_A_VALUE);
#elif defined(XNucleoV1)
	//Both Acc and Gyro are initialized while Gyro configuration
	configureGyro(hi2c, LSM6DS0_GYRO_CTRL_REG_1, LSM6DS0_GYRO_CTRL_REG_1_VALUE);
	configureMag(hi2c, LIS3MDL_MAG_CFG_REG_1, LIS3MDL_MAG_TEMP_SPEEDMODE);
	configureMag(hi2c, LIS3MDL_MAG_CFG_REG_3, LIS3MDL_MAG_CONTINOUSMODE);
	configureMag(hi2c, LIS3MDL_MAG_CFG_REG_4, LIS3MDL_MAG_Z_AXISMODE_ENDIANESS);
#else
	configureAcc(hi2c, ACC_CTRL_REG1, ACC_CTRL_REG1_200HZ);
	configureAcc(hi2c, ACC_CTRL_REG4, ACC_CTRL_REG4_2G_HR);
	configureMag(hi2c, MAG_CRA_REG, MAG_CRA_REG_30HZ);
	configureMag(hi2c, MAG_MR_REG, MAG_MR_REG_CONT);
#endif
}

void IMUSensor::initializeSPI_Sensors(SPI_HandleTypeDef *hspi)
{
#ifdef XNucleoV3

#elif defined(XNucleoV1)

#else
	configureGyro(hspi, GYRO_CNT_REG_1, GYRO_TURN_ON);
#endif
}

void IMUSensor::addToBuffer(DataBuffer<IMUData> &buffer, uint32_t timeStamp, uint8_t size)
{
	IMUData measurement(rawData, timeStamp, size);
	buffer.put(measurement);
}

void IMUSensor::pullAccData(I2C_HandleTypeDef *hi2c, uint8_t accDeviceAddr, uint8_t registerAddr, uint8_t size)
{
	if(HAL_I2C_Mem_Read(hi2c, accDeviceAddr, registerAddr, 1, rawData, size, 1) == HAL_OK)
	{
		dataTimeStamp = HAL_GetTick();

		for (uint8_t i = 0; i < 6; i += 2)
		{
			std::swap(rawData[i], rawData[i + 1]);
		}

		addToBuffer(accBuffer, dataTimeStamp, size);
	}
}

void IMUSensor::pullGyroData(I2C_HandleTypeDef *hi2c, uint8_t gyroDeviceAddr, uint8_t registerAddr, uint8_t size)
{
	if(HAL_I2C_Mem_Read(hi2c, gyroDeviceAddr, registerAddr, 1, rawData, size, 1) == HAL_OK)
	{
		dataTimeStamp = HAL_GetTick();

		for(uint8_t i = 0; i < 6; i+=2)
		{
		  std::swap(rawData[i], rawData[i + 1]);
		}

		addToBuffer(gyroBuffer, dataTimeStamp, size);
	}
}

void IMUSensor::pullGyroData(SPI_HandleTypeDef *hspi, uint8_t gyroDataAddr, uint8_t size)
{
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(hspi, &gyroDataAddr, 1, 1);
	  HAL_SPI_Receive(hspi, rawData, size, 1);
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);

	  dataTimeStamp = HAL_GetTick();

	  for(uint8_t i = 0; i < 6; i+=2)
	  {
		  std::swap(rawData[i], rawData[i + 1]);
	  }

	  addToBuffer(gyroBuffer, dataTimeStamp, size);
}

void IMUSensor::pullMagData(I2C_HandleTypeDef *hi2c, uint8_t magDeviceAddr, uint8_t registerAddr, uint8_t size)
{
	if(HAL_I2C_Mem_Read(hi2c, magDeviceAddr, registerAddr, 1, rawData, size, 1) == HAL_OK)
	{
		dataTimeStamp = HAL_GetTick();

	#ifdef XNucleoV3
		for(uint8_t i = 0; i < 6; i+=2)
		{
			std::swap(rawData[i], rawData[i + 1]);
		}
	#elif defined(XNucleoV1)
		for(uint8_t i = 0; i < 6; i+=2)
		{
			std::swap(rawData[i], rawData[i + 1]);
		}
	#else
		std::swap(rawData[2], rawData[4]);
		std::swap(rawData[3], rawData[5]);
	#endif

		addToBuffer(magBuffer, dataTimeStamp, size);
	}
}

void IMUSensor::pullDataFromSensorsI2C(I2C_HandleTypeDef *hi2c)
{
	if(!readingInProgress)
	{
	#ifdef XNucleoV3
		pullAccData(hi2c, LSM6DSO_ACC_SENS_ADDR, LSM6DSO_ACC_DATA, LSM6DSO_ACC_DATA_SIZE);
		pullGyroData(hi2c, LSM6DSO_GYRO_SENS_ADDR, LSM6DSO_GYRO_DATA, LSM6DSO_GYRO_DATA_SIZE);
		pullMagData(hi2c, LIS2MDL_MAG_SENS_ADDR, LIS2MDL_MAG_DATA | LIS2MDL_MAG_MULTI_READ, LIS2MDL_MAG_DATA_SIZE);
	#elif defined(XNucleoV1)
		pullAccData(hi2c, LSM6DS0_ACC_SENS_ADDR, LSM6DS0_ACC_DATA, LSM6DS0_ACC_DATA_SIZE);
		pullGyroData(hi2c, LSM6DS0_GYRO_SENS_ADDR, LSM6DS0_GYRO_DATA, LSM6DS0_GYRO_DATA_SIZE);
		pullMagData(hi2c, LIS3MDL_MAG_SENS_ADDR, LIS3MDL_MAG_DATA, LIS3MDL_MAG_DATA_SIZE);
	#else
		pullAccData(hi2c, ACC_SENS_ADDR, ACC_DATA | ACC_MULTIBYTE_READ,	ACC_DATA_SIZE);
		pullMagData(hi2c, MAG_SENS_ADDR, MAG_DATA, MAG_DATA_SIZE);
	#endif
	}
}

void IMUSensor::pullDataFromSensorsSPI(SPI_HandleTypeDef *hspi)
{
	if(!readingInProgress)
	{
	#ifdef XNucleoV3

	#elif defined(XNucleoV1)

	#else
		pullGyroData(hspi, GYRO_DATA | GYRO_MULTICONT_READ, GYRO_DATA_SIZE);
	#endif
	}
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
#ifdef XNucleoV3
	writeI2C(hi2c, LSM6DSO_ACC_SENS_ADDR, &accRegAddr, &regValue);

#elif defined(XNucleoV1)
	writeI2C(hi2c, LSM6DS0_ACC_SENS_ADDR, &accRegAddr, &regValue);
#else
	writeI2C(hi2c, ACC_SENS_ADDR, &accRegAddr, &regValue);
#endif
}

void IMUSensor::configureGyro(SPI_HandleTypeDef *hspi, uint8_t gyroRegAddr, uint8_t regValue)
{
	writeSPI(hspi, &gyroRegAddr, &regValue);
}

void IMUSensor::configureGyro(I2C_HandleTypeDef *hi2c, uint8_t gyroRegAddr, uint8_t regValue)
{
#ifdef XNucleoV3
	writeI2C(hi2c, LSM6DSO_GYRO_SENS_ADDR, &gyroRegAddr, &regValue);
#elif defined(XNucleoV1)
	writeI2C(hi2c, LSM6DS0_GYRO_SENS_ADDR, &gyroRegAddr, &regValue);
#else

#endif
}

void IMUSensor::configureMag(I2C_HandleTypeDef *hi2c, uint8_t magRegAddr, uint8_t regValue)
{
#ifdef XNucleoV3
	writeI2C(hi2c, LIS2MDL_MAG_SENS_ADDR, &magRegAddr, &regValue);
#elif defined(XNucleoV1)
	writeI2C(hi2c, LIS3MDL_MAG_SENS_ADDR, &magRegAddr, &regValue);
#else
	writeI2C(hi2c, MAG_SENS_ADDR, &magRegAddr, &regValue);
#endif
}

void IMUSensor::accRemoteConfiguration(uint8_t *dataBuffer)
{
	configureAcc(i2cHandle, dataBuffer[0], dataBuffer[1]);
}

void IMUSensor::gyroRemoteConfiguration(uint8_t *dataBuffer)
{
	configureGyro(i2cHandle, dataBuffer[0], dataBuffer[1]);
}

void IMUSensor::magRemoteConfiguration(uint8_t *dataBuffer)
{
	configureMag(i2cHandle, dataBuffer[0], dataBuffer[1]);
}

uint16_t IMUSensor::getBufferDataInArray(DataBuffer<IMUData> &buffer, uint8_t *dataBuffer)
{
	readingInProgress = 1;
	uint8_t numberOfElements = buffer.size();
	if(!numberOfElements)
	{
		readingInProgress = 0;
		return 0;
	}
	IMUData tempData = buffer.get();
	uint8_t ptrToArrayData[10];
	uint8_t numberOfBytesToCopy = tempData.getDataInArray(ptrToArrayData);
	std::unique_ptr<uint8_t[]> tempBuffer = std::make_unique<uint8_t[]>(numberOfElements * numberOfBytesToCopy);

	std::copy_n(ptrToArrayData, numberOfBytesToCopy, tempBuffer.get());

	for (uint16_t i = 1; i < numberOfElements; i++)
	{
		tempData = buffer.get();
		numberOfBytesToCopy = tempData.getDataInArray(ptrToArrayData);
		std::copy_n(ptrToArrayData, numberOfBytesToCopy, tempBuffer.get() + i * numberOfBytesToCopy);
	}

	std::move(tempBuffer.get(), tempBuffer.get() + numberOfElements * numberOfBytesToCopy, dataBuffer);

	readingInProgress = 0;
	return numberOfElements * numberOfBytesToCopy;
}

uint16_t IMUSensor::getAccData(uint8_t *dataBuffer)
{
	return getBufferDataInArray(accBuffer, dataBuffer);
}

uint16_t IMUSensor::getGyroData(uint8_t *dataBuffer)
{
	return getBufferDataInArray(gyroBuffer, dataBuffer);
}

uint16_t IMUSensor::getMagData(uint8_t *dataBuffer)
{
	return getBufferDataInArray(magBuffer, dataBuffer);
}
