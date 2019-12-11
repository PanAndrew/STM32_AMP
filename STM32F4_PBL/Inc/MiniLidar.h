/*
 * MiniLidar.h
 *
 *  Created on: Dec 11, 2019
 *      Author: piotrek
 */

#ifndef MINILIDAR_H_
#define MINILIDAR_H_

#include "stdint.h"
#include <algorithm>

#define BYTES_TO_ENCODE 9
#define MINI_LIDAR_OBJECTDATAVOLUME 5

class MiniLidar {

	bool updated = false;
	bool frameStarted = false;

	uint16_t distance;
	uint16_t signalStrength;
	uint8_t lidarMode;

	bool calculateAndMatchCRC(uint8_t* dataBuffer);

public:
	MiniLidar();
	virtual ~MiniLidar();

	void encode(uint8_t* dataBuffer, uint16_t bufferSize);

	uint16_t getDistance();
	uint16_t getSignalStrength();
	uint8_t getLidarMode();

	bool isUpdated();
	uint8_t getDataInArray(uint8_t* dataBuffer);

};

#endif /* MINILIDAR_H_ */
