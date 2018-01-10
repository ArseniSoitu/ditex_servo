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
	rcc_periph_clock_enable(RCC_USART2);
	usart_disable(USART2);

	usart_set_baudrate(USART2, 57600);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_CR2_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

//	USART2_CR3 |= USART_CR3_HDSEL;
//	USART_CR2(USART2) &= ~USART_CR2_LINEN;
//	USART_CR2(USART2) &= ~USART_CR2_CLKEN;
//	USART_CR3(USART2) &= ~USART_CR3_SCEN;
//	USART_CR3(USART2) &= ~USART_CR3_IREN;

	usart_enable_rx_interrupt(USART2);

	usart_enable(USART2);

	nvic_set_priority(NVIC_USART2_IRQ, 0);
	nvic_enable_irq(NVIC_USART2_IRQ);
}

static void gpioSetup(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);

	//gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO2);

	gpio_set_mode(GPIOA, GPIO_CNF_OUTPUT_OPENDRAIN, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO2);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO3);

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
	usart_set_baudrate(USART2, baud);
}

void SerialMessagesSend(const void *buf, uint8_t size)
{
	char data[100];
	char *ptr = data;
	uint8_t i;

	memcpy(data, buf, size);

//	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO2);
	for(i = 0; i < size ; i++)
	{
		usart_wait_send_ready(USART2);
		usart_send_blocking(USART2, (uint16_t) (*ptr));
		ptr++;
	}
//	usart_wait_send_ready(USART2);
	while(!usart_get_flag(USART2, USART_SR_TC)) {
	}
//	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO2);
}

void SerialNumberSend(uint32_t num)
{
	char data[10];
	uint8_t i;

	sprintf(data,"%d",(int)num);

	for(i = 0; i < strlen(data) ; i++)
	{
		usart_wait_send_ready(USART2);
		usart_send_blocking(USART2, (uint16_t) data[i]);
	}
	backSpaceSize = strlen(data);
}

void BackSpace(void)
{
	SerialMessagesSend("Time elapsed:", sizeof("Time elapsed:"));
}

void SerialSymbolSend(uint8_t sym)
{
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO2);
	usart_wait_send_ready(USART2);
	usart_send_blocking(USART2, (uint16_t) sym);
	usart_wait_send_ready(USART2);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO2);
}

void usart2_isr(void)
{
	uint8_t temp;
	if(usart_get_flag(USART2, USART_SR_RXNE))
	{
		temp = usart_recv(USART2);
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
		USART_SR(USART2) &= ~USART_SR_RXNE;
	}
}

