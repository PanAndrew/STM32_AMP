/*
 * DataBuffer.cpp
 *
 *  Created on: May 25, 2019
 *      Author: piotrek
 */

#include "DataBuffer.h"
#include "IMUData.h"
#include "DrivingCommand.h"
#include "unions.h"
#include "ADASManager.h"

template <class T>
DataBuffer<T>::DataBuffer(uint8_t size): buffer(std::unique_ptr<T[]>(new T[size])), maxSize(size) {
	// TODO Auto-generated constructor stub

}

template <class T>
DataBuffer<T>::~DataBuffer() {
	// TODO Auto-generated destructor stub
}
template <class T>
void DataBuffer<T>::put(T item)
{
	buffer[head] = item;

	if(fullFlag) {
		tail = (tail + 1) % maxSize;
	}

	head = (head + 1) % maxSize;

	fullFlag = head == tail;
}

template <class T>
T DataBuffer<T>::get()
{
	if(empty()) {
		return T();
	}

	//Read data and advance the tail (we now have a free space)
	auto val = buffer[tail];
	fullFlag = false;
	tail = (tail + 1) % maxSize;

	return val;
}

template <class T>
void DataBuffer<T>::reset()
{
	head = tail;
	fullFlag = false;
}

template <class T>
bool DataBuffer<T>::empty() const
{
	//if head and tail are equal, we are empty
	return (!fullFlag && (head == tail));
}

template <class T>
bool DataBuffer<T>::full() const
{
	//If tail is ahead the head by 1, we are full
	return fullFlag;
}

template <class T>
uint16_t DataBuffer<T>::capacity() const
{
	return maxSize;
}

template <class T>
uint16_t DataBuffer<T>::size() const
{
	uint16_t size = maxSize;

	if (!fullFlag) {
		if (head >= tail) {
			size = head - tail;
		} else {
			size = maxSize + head - tail;
		}
	}

	return size;
}

template <class T>
T DataBuffer<T>::getOfIndex(uint16_t index)
{
	return buffer[(tail + index) % maxSize];
}

template class DataBuffer<IMUData>;
template class DataBuffer<DrivingCommand>;
template class DataBuffer<floatUnion>;
template class DataBuffer<ADASManager>;
