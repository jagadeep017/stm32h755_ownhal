/*
 * LED2 toggle in push-pull mode and open drain
 */

#include <stdint.h>
#include <HAL_GPIO.h>
#include <HAL_BUS.h>
#include <HAL_RCC.h>

#define LED2_PIN_NUMBER	GPIO_PIN_1
#define LED2_PORT		GPIOE

void delay(void){
	for(uint32_t i=0; i<500000; i++);
}

int main(void)
{
	GPIO_init_t led	= {0};
	led.Pin = LED2_PIN_NUMBER;
	led.Pin_Mode = MODE_OUTPUT;
	led.Speed = GPIO_SPEED_FAST;

	//open_DRAIN	task2
//	led.Pull = GPIO_PIN_PU;
//	led.Out_Type = GPIO_OUT_OD;

	//push-pull		task1
	led.Pull = GPIO_NO_PUPD;
	led.Out_Type = GPIO_OUT_PP;

	HAL_GPIO_clk_control(LED2_PORT, ENABLE);

	HAL_GPIO_init(LED2_PORT, &led);
    /* Loop forever */
	while(1){
		HAL_GPIO_toggle_pin(LED2_PORT, PIN_1);
		delay();
	}
}
