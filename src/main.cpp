#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/nvic.h>
#include "drivers/SystemTimer.h"
#include "drivers/SerialMessages.h"

 
#define PORT_LED       GPIOB
#define PIN_LED        GPIO4
#define DEV_ADDR       0x29

#define POLY 0x07

static void ledSetup(void);
static void gpioSetup(void);
static uint8_t rccSetup(void);
static void setMco( void );
static void timSetup(void);

static uint16_t crc_ccitt_update( uint16_t crc, uint8_t data );
static uint16_t get_crc16z(uint8_t *p, uint16_t len);
unsigned char update_crc (unsigned char crc, unsigned char crc_seed);
unsigned char crc8 (unsigned char *crc, unsigned char crc_lenght);

uint32_t worktime;
uint16_t crcBuff;
uint8_t crcNew;
uint32_t lux;
uint32_t luxRaw;
uint64_t timeNow;
uint8_t dataUart = 0xFF;
uint8_t tempBuff[] = {0x2B, 0x00, 0x04, 0x02, 0x1E};
uint8_t tempBuff1[] = {0x2C, 0x00, 0xB8, 0x02, 0x52, 0xA4, 0x7D};
 
int main(void)
{
	if (!rccSetup()) {
		while (1);
	}
	SystemTimerInit();
	SerialMessagesInit();
//	crcNew = crc8(tempBuff, (sizeof(tempBuff) - 1));
//	tempBuff[4] = crcNew;

//	gpioSetup();
//	worktime = GetCurrentTime();
//	while((GetCurrentTime() - worktime) < 1200)
//	{
//
//	}
//	gpio_clear(GPIOA, GPIO2);
//	for(int i = 0; i < 22; i++) {
//		__asm__("nop");
//	}
	//SerialMessagesSend(tempBuff, sizeof(tempBuff));
//	SerialMessagesSend("******************************\n", sizeof("******************************\n"));
//	SerialMessagesSend("Servo board STM32F103TB based\n", sizeof("Servo board STM32F103TB based\n"));
//	SerialMessagesSend("******************************\n", sizeof("******************************\n"));
//	ledSetup();
	timeNow = GetCurrentTime();
    while (1) {
//    	__asm__("nop");
    	if ((GetCurrentTime() - timeNow) > 250000) {
    		timeNow = GetCurrentTime();
    		SerialMessagesSend(tempBuff1, sizeof(tempBuff1));
    	}
	}

	return 0;
}

static void timSetup(void)
{
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO2);
	timer_reset(TIM2);
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM2, 7);
    timer_set_repetition_counter(TIM2, 0);
    timer_enable_preload(TIM2);
    timer_continuous_mode(TIM2);
    timer_set_period(TIM2, 8000);

    timer_disable_oc_output(TIM2, TIM_OC3);
    timer_set_oc_mode(TIM2, TIM_OC3, TIM_OCM_PWM1);
    timer_set_oc_value(TIM2, TIM_OC3, 1500);
    timer_enable_oc_output(TIM2, TIM_OC3);

    timer_enable_counter(TIM2);
}

static void ledSetup(void)
{
	AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_set_mode(PORT_LED, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PIN_LED);

	gpio_set(PORT_LED, PIN_LED);
	gpio_clear(PORT_LED, PIN_LED);
}

static void gpioSetup(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);
	gpio_set(GPIOA, GPIO2);
}

static uint8_t rccSetup(void)
{
	if (rcc_system_clock_source() != RCC_CFGR_SWS_SYSCLKSEL_HSECLK) {
		rcc_osc_on( RCC_HSE );
		rcc_wait_for_osc_ready( RCC_HSE );
		rcc_css_enable();
		rcc_osc_bypass_enable( RCC_HSE );

		//rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_DIV2);
		rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);
		rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_NODIV);
		rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);
		flash_set_ws(FLASH_ACR_LATENCY_0WS);

		rcc_set_sysclk_source( RCC_CFGR_SW_SYSCLKSEL_HSECLK );
	}
	if (rcc_system_clock_source() == RCC_CFGR_SWS_SYSCLKSEL_HSECLK) {
		rcc_periph_clock_enable(RCC_AFIO);
		return 1;
	}
	else {
		return 0;
	}
}

static void setMco(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_MODE_OUTPUT_50_MHZ, GPIO8);
	rcc_set_mco(RCC_CFGR_MCO_SYSCLK);
}

uint16_t crc_ccitt_update( uint16_t crc, uint8_t data )
{
    uint16_t ret_val;
    data ^= (uint8_t)(crc) & (uint8_t)(0xFF);
    data ^= data << 4;
    ret_val = ((((uint16_t)data << 8) | ((crc & 0xFF00) >> 8)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
    return ret_val;
}

uint16_t get_crc16z(uint8_t *p, uint16_t len)
{
    uint16_t crc16_data=0;
    while(len--) { crc16_data=crc_ccitt_update(crc16_data, p[0]); p++; }
    return(crc16_data);
}

/* 8-bit CRC polynomial
#define POLY 0x07
X^8 + X^2 + X + 1 */
unsigned char update_crc (unsigned char crc, unsigned char crc_seed)
{
    unsigned char crc_u;
    unsigned char i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for (i=0; i<8; i++)
    {
    crc_u = ( crc_u & 0x80 ) ? POLY ^ ( crc_u << 1 ) : ( crc_u << 1 );
    }
    return crc_u;
}
unsigned char crc8 (unsigned char *crc, unsigned char crc_lenght)
{
    unsigned char crc_up = 0;
    unsigned char c;
    for(c=0;c < crc_lenght; c++) {
    crc_up = update_crc (crc[c], crc_up);
    }
    return crc_up;
}


