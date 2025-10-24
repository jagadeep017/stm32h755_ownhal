/*
 * HAL_GPIO.c
 *
 *  Created on: Oct 1, 2025
 *      Author: jagadeep.g
 */

#include <HAL_GPIO.h>
#include <HAL_RCC.h>
#include <HAL_EXTI.h>
#include <HAL_SYSCFG.h>
#include <HAL_IRQ.h>


/*
 * GPIO init
 * GPIOx	:-	base address of GPIO registers (GPIOA-GPIOK)
 * gpio_init:-	parameters for config
 */;

void HAL_GPIO_init(GPIO_reg_t * GPIOx, GPIO_init_t * gpio_init){
	uint8_t position =0x00U;
	uint32_t iocurrent;
	uint32_t temp;		//buffer for gpio bit processing
	//need to implement assertions
	while(gpio_init->Pin>>position){
		if((iocurrent = ((gpio_init->Pin)&(1<<position)))){

			if(gpio_init->Pin_Mode <= MODE_ANALOG){
				//setting IO type
				//clearing the bit
				temp = (GPIOx->MODER)&(~(GPIO_MODE(position)));
				temp |= gpio_init->Pin_Mode<<(2*position);
				GPIOx->MODER = temp;
				if(gpio_init->Pin_Mode==MODE_OUTPUT || gpio_init->Pin_Mode==MODE_AF){
					//setting the GPIO SPEED
					//clearing the 2 bit in OSPEEDR of that pins
					temp = (GPIOx->OSPEEDR)&(~(GPIO_OSPEDR_OSPEED0<<(position*2)));
					//setting the 2 bits
					temp |= gpio_init->Speed<<(position*2U);
					GPIOx->OSPEEDR = temp;

					//setting output type register
					//clearing the bit
					temp = (GPIOx->OTYPER)&(~(GPIO_OTYPER_OT0<<position));
					//setting the bit
					temp |= gpio_init->Out_Type<<position;
					GPIOx->OTYPER = temp;
					if(gpio_init->Pin_Mode == MODE_AF){
						uint8_t temp1 = position/8;	//indexing AFR register low or high
						uint8_t temp2 = position%8;	//for location of the bit fields of the pin in AFR
						temp = (GPIOx->AFR[temp1])&(~(GPIO_AFR_0)<<(4U*temp2));
						temp |=	gpio_init->Alternate<<(4U*temp2);
						GPIOx->AFR[temp1] |= temp;
					}
				}
			}
			else{
				temp = (GPIOx->MODER)&(~(GPIO_MODE(position)));
				temp |= MODE_INPUT<<(2*position);
				GPIOx->MODER = temp;
				//interrupt types
				if(gpio_init->Pin_Mode == MODE_INT_FALL_EDGE){
					//enabling falling edge trigger
					EXTI->FTSR1 |= (1<<position);
					//clearing rising edge trigger just in case
					EXTI->RTSR1 &= ~(1<<position);
				}
				else if(gpio_init->Pin_Mode == MODE_INT_RISE_EDGE){
					//enabling raising edge trigger
					EXTI->RTSR1 |= (1<<position);
					//clearing falling edge trigger just in case
					EXTI->FTSR1 &= ~(1<<position);
				}
				else if(gpio_init->Pin_Mode == MODE_INT_FALL_RISE_EDGE){
					//enabling falling edge trigger and rising edge trigger
					EXTI->FTSR1 |= (1<<position);
					EXTI->RTSR1 |= (1<<position);
				}

				//config GPIO port selection in SYSCFG_EXTICR
				uint32_t temp2 = position/4, temp3 = position%4, portcode;
				//enabling clock for SYSCFG
				SYSCFG_CLK_EN();
				//clear the 4 bit first
				SYSCFG->EXTICR[temp2] &= ~(0xFU<<temp3*4);

				portcode = GPIO_TO_PORTCODE(GPIOx);
				//setting the required GPIO port
				SYSCFG->EXTICR[temp2] |= (portcode<<temp3*4);

				//enabling the interrupt delivery using IMR
				EXTI->C1IMR1 |= (1<<position);
			}
			//configuring pull up/down
			//clearing the 2 bits in PUPDR of that pins
			temp = (GPIOx->PUPDR)&(~(GPIO_PUPDR_0<<(2*position)));
			//setting the 2 bits
			temp|= gpio_init->Pull<<(position*2U);
			GPIOx->PUPDR = temp;
}
		position++;
	}
}


/*
 * GPIO deinit reset all the GPIO registers
 * GPIOx	:-	base address of GPIO registers (GPIOA-GPIOK)
 */;
void HAL_GPIO_deinit(GPIO_reg_t *GPIOx){
	if(GPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(GPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if(GPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if(GPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if(GPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if(GPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
	else if(GPIOx == GPIOG){
		GPIOG_REG_RESET();
	}
	else if(GPIOx == GPIOH){
		GPIOH_REG_RESET();
	}
	else if(GPIOx == GPIOI){
		GPIOI_REG_RESET();
	}
	else if(GPIOx == GPIOK){
		GPIOK_REG_RESET();
	}
}


/*
 * GPIO clock setup
 * GPIOx	:-	base address of GPIO registers (GPIOA-GPIOK)
 * type		:-	macro for enable(1) or disable(0)
 */
void HAL_GPIO_clk_control(GPIO_reg_t *GPIOx, uint8_t type){
	if(type == ENABLE){
		if(GPIOx == GPIOA){
			GPIOA_CLK_EN();
		}
		else if(GPIOx == GPIOB){
			GPIOB_CLK_EN();
		}
		else if(GPIOx == GPIOC){
			GPIOC_CLK_EN();
		}
		else if(GPIOx == GPIOD){
			GPIOD_CLK_EN();
		}
		else if(GPIOx == GPIOE){
			GPIOE_CLK_EN();
		}
		else if(GPIOx == GPIOF){
			GPIOF_CLK_EN();
		}
		else if(GPIOx == GPIOG){
			GPIOG_CLK_EN();
		}
		else if(GPIOx == GPIOH){
			GPIOH_CLK_EN();
		}
		else if(GPIOx == GPIOI){
			GPIOI_CLK_EN();
		}
		else if(GPIOx == GPIOK){
			GPIOK_CLK_EN();
		}
	}
	else{
		if(GPIOx == GPIOA){
			GPIOA_CLK_DI();
		}
		else if(GPIOx == GPIOB){
			GPIOB_CLK_DI();
		}
		else if(GPIOx == GPIOC){
			GPIOC_CLK_DI();
		}
		else if(GPIOx == GPIOD){
			GPIOD_CLK_DI();
		}
		else if(GPIOx == GPIOE){
			GPIOE_CLK_DI();
		}
		else if(GPIOx == GPIOF){
			GPIOF_CLK_DI();
		}
		else if(GPIOx == GPIOG){
			GPIOG_CLK_DI();
		}
		else if(GPIOx == GPIOH){
			GPIOH_CLK_DI();
		}
		else if(GPIOx == GPIOI){
			GPIOI_CLK_DI();
		}
		else if(GPIOx == GPIOK){
			GPIOK_CLK_DI();
		}
	}
}


/*
 * GPIO read pin
 * GPIOx	:-	base address of GPIO registers (GPIOA-GPIOK)
 * pin_num	:- macro for pin 0-15(GPIO_PIN_0-15) @GPIO_PIN_NUMBER
 */
uint8_t HAL_GPIO_read_pin(GPIO_reg_t *GPIOx, uint8_t pin_num){
	return (uint8_t)((GPIOx->IDR>>pin_num) & 0x01);
}

/*
 * GPIO read port
 * GPIOx	:-	base address of GPIO registers (GPIOA-GPIOK)
 */
uint16_t HAL_GPIO_read_port(GPIO_reg_t *GPIOx){
	return (uint16_t)(GPIOx->IDR);
}

/*
 * GPIO write pin
 * GPIOx	:-	base address of GPIO registers (GPIOA-GPIOK)
 * pin_num	:- macro for pin 0-15(GPIO_PIN_0-15) @GPIO_PIN_NUMBER
 * value	:-	value to write to the pin
 */
void HAL_GPIO_write_pin(GPIO_reg_t *GPIOx, uint8_t pin_num, uint8_t value){
	if(value == SET){
		GPIOx->ODR |= (1<<pin_num);
	}
	else{
		GPIOx->ODR &= ~(1<<pin_num);
	}
}

/*
 * GPIO write port
 * GPIOx	:-	base address of GPIO registers (GPIOA-GPIOK)
 * value	:-	value to write to the pin
 */
void HAL_GPIO_write_port(GPIO_reg_t *GPIOx, uint16_t value){
	GPIOx->ODR = value;
}

/*
 * GPIO toggle output pin
 * GPIOx	:-	base address of GPIO registers (GPIOA-GPIOK)
 * pin_num	:- macro for pin 0-15(GPIO_PIN_0-15) @GPIO_PIN_NUMBER
 */
void HAL_GPIO_toggle_pin(GPIO_reg_t *GPIOx, uint8_t pin_num){
	GPIOx->ODR ^= (1<<pin_num);
}


void HAL_GPIO_irq_handler(uint8_t pin_num){
	if(EXTI->C1PR1 & (1<<pin_num)){
		EXTI->C1PR1 |= 1<<pin_num;
	}
}

