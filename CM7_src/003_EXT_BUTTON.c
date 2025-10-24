/*
 * 003_EXT_BUTTON.c
 *
 *  Created on: Oct 3, 2025
 *      Author: jagadeep.g
 */



#include <stdint.h>
#include <HAL_GPIO.h>
#include <HAL_BUS.h>
#include <HAL_RCC.h>
#include <HAL_IRQ.h>

#define HIGH	1
#define LOW		0

#define LED2_PIN_NUMBER	GPIO_PIN_1
#define LED2_PORT		GPIOE
#define EXT_BUTTON_PORT	GPIOE
#define USER_BUTTON_PORT	GPIOC


void delay(void){
	for(uint32_t i=0; i<500000; i++); //could be better
}

void EXTI15_10_IRQHandler(void){	//C13 is user button interrupt E13 extern button

	HAL_GPIO_irq_handler(PIN_13);
	HAL_GPIO_toggle_pin(LED2_PORT, PIN_1);
	delay();
}


int main(void)
{
	GPIO_init_t led	= {0},button = {0};
	led.Pin = LED2_PIN_NUMBER;
	led.Pin_Mode = MODE_OUTPUT;
	led.Speed = GPIO_SPEED_FAST;
	led.Pull = GPIO_NO_PUPD;
	led.Out_Type = GPIO_OUT_PP;

	//USER button config
	button.Pin = GPIO_PIN_13;		//PORTC 13pin
	button.Pin_Mode = MODE_INT_RISE_EDGE;
	button.Pull = GPIO_NO_PUPD;

	//EXTERNAL BUTTON config
//	button.Pin = GPIO_PIN_13;		//PORTE 13 pin
//	button.Pin_Mode = MODE_INT_FALL_EDGE;
//	button.Pull = GPIO_PIN_PU;

	HAL_GPIO_clk_control(LED2_PORT, ENABLE);

	HAL_GPIO_clk_control(USER_BUTTON_PORT, ENABLE);

	HAL_GPIO_init(LED2_PORT, &led);

	HAL_GPIO_init(USER_BUTTON_PORT, &button);

	HAL_GPIO_irq_config(IRQ_NO_EXTI15_10, ENABLE);	//irq number of EXTI13(10-15) is 40

	HAL_GPIO_irq_priority(IRQ_NO_EXTI15_10, IRQ_PRI15);	//15 is lowest priority

    /* Loop forever */
	while(1){

	}
}
