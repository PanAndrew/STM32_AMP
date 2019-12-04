/*
 * DataBuffer.h
 *
 *  Created on: May 25, 2019
 *      Author: piotrek
 */

#ifndef DATABUFFER_H_
#define DATABUFFER_H_

#include <memory>

template <class T> class DataBuffer {

	std::unique_ptr<T[]> buffer;
	uint8_t head = 0;
	uint8_t tail = 0;
	const uint8_t maxSize;
	bool fullFlag = 0;

public:
	DataBuffer(uint8_t size);
	virtual ~DataBuffer();

	void put(T item);
	T get();
	void reset();
	bool empty() const;
	bool full() const;
	uint16_t capacity() const;
	uint16_t size() const;
	T getOfIndex(uint16_t index);
};

#endif /* DATABUFFER_H_ */
