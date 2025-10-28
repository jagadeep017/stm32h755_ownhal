/*
 * HAL_IRQ.h
 *
 *  Created on: Oct 6, 2025
 *      Author: jagadeep.g
 */

#ifndef STM32H7XX_HAL_INC_HAL_IRQ_H_
#define STM32H7XX_HAL_INC_HAL_IRQ_H_

#include <stdint.h>


#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84
#define IRQ_NO_SPI5			85
#define IRQ_NO_SPI6			86


#define IRQ_PRI0			0
#define IRQ_PRI1			1
#define IRQ_PRI2			2
#define IRQ_PRI3			3
#define IRQ_PRI4			4
#define IRQ_PRI5			5
#define IRQ_PRI6			6
#define IRQ_PRI7			7
#define IRQ_PRI8			8
#define IRQ_PRI9			9
#define IRQ_PRI10			10
#define IRQ_PRI11			11
#define IRQ_PRI12			12
#define IRQ_PRI13			13
#define IRQ_PRI14			14
#define IRQ_PRI15			15


/*
 * ARM cortex-m7 processor NVIC ISERx REGISTER ADDRESS
 * interrupt set-enable register
 */
#define NVIC_ISER		((volatile uint32_t*)0xE000E100UL)	//NVIC_ISER[x] x:0-7


/*
 * ARM cortex-m7 processor NVIC ICERx REGISTER ADDRESS
 * interrupt clear-enable register
 */
#define NVIC_ICER		((volatile uint32_t*)0XE000E180UL)	//NVIC_ICER[x] x:0-7


/*
 * ARM cortex-m7 processor NVIC IPRx REGISTOR ADDRESS
 * interrupt priority register
 */
#define NVIC_IPR		((volatile uint32_t*)0xE000E400UL)	//NVIC_IPR[x] x:0-59


void HAL_irq_config(uint8_t irq_number, uint8_t type);


#endif /* STM32H7XX_HAL_INC_HAL_IRQ_H_ */
