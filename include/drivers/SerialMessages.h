/*
 * SerialMessages.h
 *
 *  Created on: Nov 10, 2017
 *      Author: Arseniy Soytu
 */

#ifndef INCLUDE_SERIALMESSAGES_H_
#define INCLUDE_SERIALMESSAGES_H_

void SerialMessagesInit(void);
void SerialMessagesSend(const void *buf, uint8_t size);
void SerialNumberSend(uint32_t num);
void SerialSymbolSend(uint8_t sym);
void BackSpace(void);
void SetBaudrate(uint32_t baud);

#endif /* INCLUDE_SERIALMESSAGES_H_ */
