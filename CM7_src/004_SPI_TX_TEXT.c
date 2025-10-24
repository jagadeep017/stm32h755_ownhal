/*
 * 004_SPI_TX_TEXT.c
 *
 *  Created on: Oct 8, 2025
 *      Author: jagadeep.g
 *  Section no: 40
 */

#include <stdint.h>
#include <string.h>
#include <HAL_GPIO.h>
#include <HAL_BUS.h>
#include <HAL_RCC.h>
#include <HAL_IRQ.h>
#include <HAL_SPI.h>

#define HIGH	1
#define LOW		0

#define LED2_PIN_NUMBER		GPIO_PIN_1
#define LED2_PORT			GPIOE
#define EXT_BUTTON_PORT		GPIOE
#define USER_BUTTON_PORT	GPIOC

void spi4_gpio_init(void){
	HAL_GPIO_clk_control(GPIOE, ENABLE);

	//SPI4 bus gpio config
	GPIO_init_t gpio = {0};

	gpio.Pin_Mode = MODE_AF;
	gpio.Speed = GPIO_SPEED_FAST;
	gpio.Alternate = SPI_AF;

	//NSS config
//	gpio.Pin = GPIO_PIN_;
//	HAL_GPIO_init(GPIO, &gpio);

	//SCLK config
	gpio.Pin = GPIO_PIN_2;
	HAL_GPIO_init(GPIOE, &gpio);

	//MISO config
//	gpio.Pin = GPIO_PIN_;
//	HAL_GPIO_init(GPIO, &gpio);

	//MOSI config
	gpio.Pin = GPIO_PIN_6;
	HAL_GPIO_init(GPIOE, &gpio);
}


void delay(void){
	for(uint32_t i=0; i<500000; i++); //could be better
}

void spi4_init(void){

	RCC->PLLCFGR |=(0x7<<4);
	RCC->PLL2DIVR &=~(0x1ff<<0);//clear
	RCC->PLL2DIVR |=(0x4f<<0);	//divn2
	RCC->PLL2DIVR |= (0x13<<16);//divq2

	RCC->CR |= (1<<26);//PLL2on

	while(!(RCC->CR & (1<<27)));

	SPI45_CLK_SEL(SPI45_PLL2_CK);

	SPI4_CLK_EN();
	SPI_config_t spi4 = {0};

	spi4_gpio_init();
	spi4.DeviceMode = SPI_MODE_MASTER;
	spi4.BusConfig = SPI_BUS_SIMPLEX_TX;
	spi4.SclkSpeed = SPI_CLK_DIV8;
	spi4.DFF = SPI_DFF_8BITS;
	spi4.SSM = SPI_SSM_SW;

	SPI_handle_t spi4_handle = {SPI4,spi4};
	HAL_SPI_init(&spi4_handle);
}


int main(void)
{

	SPI4_CLK_EN();

	spi4_init();

	char pTxbuffer[] = "Hello World";
	SPI_send(SPI4, (uint8_t*)pTxbuffer, strlen(pTxbuffer));
	/* Loop forever */
	while(1){

	}
}


