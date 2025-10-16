/*
 * HAL_GPIO.h
 *
 *  Created on: Oct 1, 2025
 *      Author: jagadeep.g
 */

#ifndef STM32H7XX_HAL_INC_HAL_GPIO_H_
#define STM32H7XX_HAL_INC_HAL_GPIO_H_
#include <stdint.h>
#include <HAL_RCC.h>



typedef struct{
	volatile uint32_t MODER;		//GPIO port mode register
	volatile uint32_t OTYPER;		//GPIO port output type register
	volatile uint32_t OSPEEDR;		//GPIO port output speed register
	volatile uint32_t PUPDR;		//GPIO port pull-up/pull-down register
	volatile uint32_t IDR;			//GPIO port input data register
	volatile uint32_t ODR;			//GPIO port output data register
	volatile uint32_t BSRR;			//GPIO port bit set/reset register
	volatile uint32_t LCKR;			//GPIO port configuration lock register
	volatile uint32_t AFR[2];		//GPIO alternate function AFR[0]=low register AFR[1]=high register
}GPIO_reg_t;

typedef struct{						//possible values
	uint32_t Pin;					//@GPIO_PIN_NUMBER_config
	uint32_t Pin_Mode;				//@GPIO_PIN_MODES
	uint32_t Pull;					//@possible_PULL_TYPES
	uint32_t Speed;					//@GPIO_pin_output_speeds
	uint32_t Alternate;				//@GPIO_AF_MODES
	uint32_t Out_Type;				//@possible_output_modes
}GPIO_init_t;

/**
 * @brief GPIO registers of stm32h755
 * connected to AHB4 BUS
 */

#define GPIOA_BASE		(0x58020000UL)
#define GPIOB_BASE		(0x58020400UL)
#define GPIOC_BASE		(0x58020800UL)
#define GPIOD_BASE		(0x58020C00UL)
#define GPIOE_BASE		(0x58021000UL)
#define GPIOF_BASE		(0x58021400UL)
#define GPIOG_BASE		(0x58021800UL)
#define GPIOH_BASE		(0x58021C00UL)
#define GPIOI_BASE		(0x58022000UL)
#define GPIOJ_BASE		(0x58022400UL)
#define GPIOK_BASE		(0x58022800UL)

#define GPIOA		((GPIO_reg_t *) GPIOA_BASE)
#define GPIOB		((GPIO_reg_t *) GPIOB_BASE)
#define GPIOC		((GPIO_reg_t *) GPIOC_BASE)
#define GPIOD		((GPIO_reg_t *) GPIOD_BASE)
#define GPIOE		((GPIO_reg_t *) GPIOE_BASE)
#define GPIOF		((GPIO_reg_t *) GPIOF_BASE)
#define GPIOG		((GPIO_reg_t *) GPIOG_BASE)
#define GPIOH		((GPIO_reg_t *) GPIOH_BASE)
#define GPIOI		((GPIO_reg_t *) GPIOI_BASE)
#define GPIOJ		((GPIO_reg_t *) GPIOJ_BASE)
#define GPIOK		((GPIO_reg_t *) GPIOK_BASE)

/*
 * GPIO register reset
 */
#define GPIOA_REG_RESET()	do{(RCC->AHB4RSTR |= (1<<0));	(RCC->AHB4RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()	do{(RCC->AHB4RSTR |= (1<<1));	(RCC->AHB4RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()	do{(RCC->AHB4RSTR |= (1<<2));	(RCC->AHB4RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()	do{(RCC->AHB4RSTR |= (1<<3));	(RCC->AHB4RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()	do{(RCC->AHB4RSTR |= (1<<4));	(RCC->AHB4RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()	do{(RCC->AHB4RSTR |= (1<<5));	(RCC->AHB4RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()	do{(RCC->AHB4RSTR |= (1<<6));	(RCC->AHB4RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()	do{(RCC->AHB4RSTR |= (1<<7));	(RCC->AHB4RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET()	do{(RCC->AHB4RSTR |= (1<<8));	(RCC->AHB4RSTR &= ~(1<<8));}while(0)
#define GPIOJ_REG_RESET()	do{(RCC->AHB4RSTR |= (1<<9));	(RCC->AHB4RSTR &= ~(1<<9));}while(0)
#define GPIOK_REG_RESET()	do{(RCC->AHB4RSTR |= (1<<10));	(RCC->AHB4RSTR &= ~(1<<10));}while(0)

#define MCO1_PORT	GPIOA
#define MCO2_PORT	GPIOC

#define MCO1_PIN	GPIO_PIN_8
#define MCO2_PIN	GPIO_PIN_9

#define MCO_AF		(0x00U)
#define SPI_AF		(0x05U)

/*
 * @GPIO_PIN_NUMBER_config
 */
#define GPIO_PIN_0	((uint16_t)0x0001U)
#define GPIO_PIN_1	((uint16_t)0x0002U)
#define GPIO_PIN_2	((uint16_t)0x0004U)
#define GPIO_PIN_3	((uint16_t)0x0008U)
#define GPIO_PIN_4	((uint16_t)0x0010U)
#define GPIO_PIN_5	((uint16_t)0x0020U)
#define GPIO_PIN_6	((uint16_t)0x0040U)
#define GPIO_PIN_7	((uint16_t)0x0080U)
#define GPIO_PIN_8	((uint16_t)0x0100U)
#define GPIO_PIN_9	((uint16_t)0x0200U)
#define GPIO_PIN_10	((uint16_t)0x0400U)
#define GPIO_PIN_11	((uint16_t)0x0800U)
#define GPIO_PIN_12	((uint16_t)0x1000U)
#define GPIO_PIN_13	((uint16_t)0x2000U)
#define GPIO_PIN_14	((uint16_t)0x4000U)
#define GPIO_PIN_15	((uint16_t)0x8000U)

/*
 * @GPIO_PIN_NUMBER
 */
#define PIN_0		0
#define PIN_1		1
#define PIN_2		2
#define PIN_3		3
#define PIN_4		4
#define PIN_5		5
#define PIN_6		6
#define PIN_7		7
#define PIN_8		8
#define PIN_9		9
#define PIN_10		10
#define PIN_11		11
#define PIN_12		12
#define PIN_13		13
#define PIN_14		14
#define PIN_15		15
/*
 * @GPIO_PIN_MODES
 */
#define GPIO_MODE(pin)			(0x03UL << (pin*2))
#define MODE_INPUT				(0x00U)
#define MODE_OUTPUT				(0x01U)
#define MODE_AF					(0x02U)
#define MODE_ANALOG				(0x03U)
#define MODE_INT_FALL_EDGE		(0x04U)	//interrupt falling edge
#define MODE_INT_RISE_EDGE		(0x05U)	//interrupt rising edge
#define MODE_INT_FALL_RISE_EDGE	(0x06U)	//interrupt rising and falling edge

/*
 * @possible_output_modes
 */
#define GPIO_OUT_PP				(0x00U)	//push-pull
#define GPIO_OUT_OD				(0x01U)	//open-drain

/*
 * @GPIO_pin_output_speeds
 */
#define GPIO_SPEED_LOW			(0x00U)
#define GPIO_SPEED_MEDIUM		(0x01U)
#define GPIO_SPEED_FAST			(0x02U)
#define GPIO_SPEED_HIGH			(0x03U)

/*
 * @possible_PULL_TYPES
 */
#define GPIO_NO_PUPD			(0x00U)
#define GPIO_PIN_PU				(0x01U)
#define GPIO_PIN_PD				(0x02U)


#define GPIO_OTYPER_OT0 (0x01U)		//used to clear bits
#define GPIO_PUPDR_0	(0x03U)
#define GPIO_OSPEDR_OSPEED0		(0x03U)
#define GPIO_AFR_0		(0x0FUL)


/*
 * GPIO init and deinit
 * GPIOx	:-	base address of GPIO registers (GPIOA-GPIOK)
 * gpio_init:-	parameters for config
 */
void HAL_GPIO_init(GPIO_reg_t *GPIOx, GPIO_init_t * gpio_init);

void HAL_GPIO_deinit(GPIO_reg_t *GPIOx);


/*
 * GPIO clock setup
 * GPIOx	:-	base address of GPIO registers (GPIOA-GPIOK)
 * type		:-	macro for enable(1) or disable(0)
 */
void HAL_GPIO_clk_control(GPIO_reg_t *GPIOx, uint8_t type);


/*
 * GPIO read pin or port
 * GPIOx	:-	base address of GPIO registers (GPIOA-GPIOK)
 * pin_num	:- macro for pin 0-15(GPIO_PIN_0-15)
 */
uint8_t HAL_GPIO_read_pin(GPIO_reg_t *GPIOx, uint8_t pin_num);

uint16_t HAL_GPIO_read_port(GPIO_reg_t *GPIOx);

/*
 * GPIO write pin or port
 * GPIOx	:-	base address of GPIO registers (GPIOA-GPIOK)
 * pin_num	:- macro for pin 0-15(GPIO_PIN_0-15)
 * value	:-	value to write to the pin
 */
void HAL_GPIO_write_pin(GPIO_reg_t *GPIOx, uint8_t pin_num, uint8_t value);

void HAL_GPIO_write_port(GPIO_reg_t *GPIOx, uint16_t value);

/*
 * GPIO toggle output pin
 * GPIOx	:-	base address of GPIO registers (GPIOA-GPIOK)
 * pin_num	:- macro for pin 0-15(GPIO_PIN_0-15)
 */
void HAL_GPIO_toggle_pin(GPIO_reg_t *GPIOx, uint8_t pin_num);

/*
 * IRQ configuration, priority and handler
 */
void HAL_GPIO_irq_config(uint8_t irq_number, uint8_t type);

void HAL_GPIO_irq_priority(uint8_t irq_number, uint8_t priority);

void HAL_GPIO_irq_handler(uint8_t pin_num);


#define GPIO_TO_PORTCODE(x)	(x==GPIOA)? 0U:\
							(x==GPIOB)? 1U:\
							(x==GPIOC)? 2U:\
							(x==GPIOD)? 3U:\
							(x==GPIOE)? 4U:\
							(x==GPIOF)? 5U:\
							(x==GPIOG)? 6U:\
							(x==GPIOH)? 7U:\
							(x==GPIOI)? 8U:\
							(x==GPIOJ)? 9U:\
							(x==GPIOK)? 10U:0U\

#endif /* STM32H7XX_HAL_INC_HAL_GPIO_H_ */
