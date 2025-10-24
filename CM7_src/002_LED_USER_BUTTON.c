/*
 * 002_LED_USER_BUTTON.c
 *
 *  Created on: Oct 3, 2025
 *      Author: jagadeep.g
 */


#include <stdint.h>
#include <HAL_GPIO.h>
#include <HAL_BUS.h>
#include <HAL_RCC.h>

#define LED2_PIN_NUMBER	GPIO_PIN_1
#define LED2_PORT		GPIOE
#define USER_BUTTON_PORT	GPIOC

void delay(void){
	for(uint32_t i=0; i<100000; i++);
}

int main(void)
{
	uint8_t prev_user_button = 0;
	GPIO_init_t led	= {0},button = {0};
	led.Pin = LED2_PIN_NUMBER;
	led.Pin_Mode = MODE_OUTPUT;
	led.Speed = GPIO_SPEED_FAST;
	led.Pull = GPIO_NO_PUPD;
	led.Out_Type = GPIO_OUT_PP;

	button.Pin = GPIO_PIN_13;
	button.Pin_Mode = MODE_INPUT;
	button.Pull = GPIO_NO_PUPD;

	HAL_GPIO_clk_control(LED2_PORT, ENABLE);

	HAL_GPIO_clk_control(USER_BUTTON_PORT, ENABLE);

	HAL_GPIO_init(LED2_PORT, &led);

	HAL_GPIO_init(USER_BUTTON_PORT, &button);
    /* Loop forever */
	while(1){
		if(HAL_GPIO_read_pin(USER_BUTTON_PORT, PIN_13)){
			if(prev_user_button == 0){
				HAL_GPIO_toggle_pin(LED2_PORT, PIN_1);
				delay();
			}
			prev_user_button = 1;
		}
		else{
			prev_user_button = 0;
		}

	}
}

