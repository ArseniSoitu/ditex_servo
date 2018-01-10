/*
 * SerialMessages.h
 *
 *  Created on: Nov 10, 2017
 *      Author: Arseniy Soytu
 */

#ifndef INCLUDE_SERIALMESSAGES_H_
#define INCLUDE_SERIALMESSAGES_H_

#include "GpioManager.hpp"

void SerialMessagesInit(void);
void SerialMessagesSend(const void *buf, uint8_t size);
void SerialNumberSend(uint32_t num);
void SerialSymbolSend(uint8_t sym);
void BackSpace(void);
void SetBaudrate(uint32_t baud);

class SerialMessage: public GpioManager {
public:
	SerialMessage(uint32_t usartParam, uint32_t baudrateParam, uint32_t gpioParam, uint16_t txPinParam, uint16_t rxPinParam):
		usart(usartParam),
		baudrate(baudrateParam),
		gpio(gpioParam),
		rxPin(rxPinParam),
		txPin(txPinParam)
    {

    }
	~SerialMessage() = default;
private:
	uint32_t usart;
	uint32_t baudrate;
	uint32_t gpio;
	uint16_t rxPin;
	uint16_t txPin;

};

#endif /* INCLUDE_SERIALMESSAGES_H_ */
