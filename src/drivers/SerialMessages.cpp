/*
 * SerialMessages.cpp
 *
 *  Created on: Nov 10, 2017
 *      Author: Arseniy Soytu
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <string.h>
#include <stdio.h>
#include "drivers/SerialMessages.h"

#define USED_USART USART3

#if USED_USART == USART3
#define RCC_USART RCC_USART3
#define NVIC_USART NVIC_USART3_IRQ
#endif

#if USED_USART == USART2
#define RCC_USART RCC_USART2
#define NVIC_USART NVIC_USART2_IRQ
#endif

static void usartSetup(void);
static void gpioSetup(void);

uint32_t backSpaceSize;
uint8_t usartBuffer[100];
uint8_t bufferCounter;
uint8_t enableBuffer;

void SerialMessagesInit(void)
{
	gpioSetup();
	usartSetup();
}

static void usartSetup(void)
{
	rcc_periph_clock_enable(RCC_USART);
	usart_disable(USED_USART);

	usart_set_baudrate(USED_USART, 57600);
	usart_set_databits(USED_USART, 8);
	usart_set_stopbits(USED_USART, USART_CR2_STOPBITS_1);
	usart_set_mode(USED_USART, USART_MODE_TX_RX);
	usart_set_parity(USED_USART, USART_PARITY_NONE);
	usart_set_flow_control(USED_USART, USART_FLOWCONTROL_NONE);

	USART3_CR3 |= USART_CR3_HDSEL;
	USART_CR2(USED_USART) &= ~USART_CR2_LINEN;
	USART_CR2(USED_USART) &= ~USART_CR2_CLKEN;
	USART_CR3(USED_USART) &= ~USART_CR3_SCEN;
	USART_CR3(USED_USART) &= ~USART_CR3_IREN;

	usart_enable_rx_interrupt(USED_USART);

	usart_enable(USED_USART);

	nvic_set_priority(NVIC_USART, 0);
	nvic_enable_irq(NVIC_USART);
}

static void gpioSetup(void)
{
	rcc_periph_clock_enable(RCC_GPIOB);

	//gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO2);

	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO10);
	//gpio_set_mode(GPIOB, GPIO_CNF_OUTPUT_OPENDRAIN, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO10);
	//gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO11);

//	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
//	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
//
//	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO2);
//	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO3);
//
//	gpio_set_af(GPIOA, GPIO_AF1, GPIO2);
//	gpio_set_af(GPIOA, GPIO_AF1, GPIO3);
}

void SetBaudrate(uint32_t baud)
{
	usart_set_baudrate(USED_USART, baud);
}

void SerialMessagesSend(const void *buf, uint8_t size)
{
	char data[100];
	char *ptr = data;
	uint8_t i;

	memcpy(data, buf, size);

	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO10);
	for(i = 0; i < size ; i++)
	{
		usart_wait_send_ready(USED_USART);
		usart_send_blocking(USED_USART, (uint16_t) (*ptr));
		ptr++;
	}
//	usart_wait_send_ready(USART2);
	while(!usart_get_flag(USED_USART, USART_SR_TC)) {
	}
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO10);
}

void SerialNumberSend(uint32_t num)
{
	char data[10];
	uint8_t i;

	sprintf(data,"%d",(int)num);

	for(i = 0; i < strlen(data) ; i++)
	{
		usart_wait_send_ready(USED_USART);
		usart_send_blocking(USED_USART, (uint16_t) data[i]);
	}
	backSpaceSize = strlen(data);
}

void BackSpace(void)
{
	SerialMessagesSend("Time elapsed:", sizeof("Time elapsed:"));
}

void SerialSymbolSend(uint8_t sym)
{
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO10);
	usart_wait_send_ready(USED_USART);
	usart_send_blocking(USED_USART, (uint16_t) sym);
	while(!usart_get_flag(USED_USART, USART_SR_TC)) {
	}
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO10);
}

void usart2_isr(void)
{
	uint8_t temp;
	if(usart_get_flag(USED_USART, USART_SR_RXNE))
	{
		temp = usart_recv(USED_USART);
		if (temp == 0x3C) {
			enableBuffer = 1;
		}
		if (enableBuffer) {
			usartBuffer[bufferCounter] = temp;
			++bufferCounter;
		}
		if (bufferCounter > 6)
		{
			bufferCounter = 0;
		}
		USART_SR(USED_USART) &= ~USART_SR_RXNE;
	}
}

void usart3_isr(void)
{
	uint8_t temp;
	if(usart_get_flag(USED_USART, USART_SR_RXNE))
	{
		temp = usart_recv(USED_USART);
		if (temp == 0x3C) {
			enableBuffer = 1;
		}
		if (enableBuffer) {
			usartBuffer[bufferCounter] = temp;
			++bufferCounter;
		}
		if (bufferCounter > 99)
		{
			bufferCounter = 0;
		}
		USART_SR(USED_USART) &= ~USART_SR_RXNE;
	}
}

