/*
 * HAL_EXTI.h
 *
 *  Created on: Oct 6, 2025
 *      Author: jagadeep.g
 */

#ifndef STM32H7XX_HAL_INC_HAL_EXTI_H_
#define STM32H7XX_HAL_INC_HAL_EXTI_H_

#include <HAL_BUS.h>
#include <stdint.h>


typedef struct{
	volatile uint32_t RTSR1;
	volatile uint32_t FTSR1;
	volatile uint32_t SWIER1;
	volatile uint32_t D3PMR1;
	volatile uint32_t D3PCR1L;
	volatile uint32_t D3PCR1H;
	uint32_t reserved3[2];
	volatile uint32_t RTSR2;
	volatile uint32_t FTSR2;
	volatile uint32_t SWIER2;
	volatile uint32_t D3PMR2;
	volatile uint32_t D3PCR2L;
	volatile uint32_t D3PCR2H;
	uint32_t reserved4[2];
	volatile uint32_t RTSR3;
	volatile uint32_t FTSR3;
	volatile uint32_t SWIER3;
	volatile uint32_t D3PMR3;
	volatile uint32_t D3PCR3L;
	volatile uint32_t D3PCR3H;
	uint32_t reserved[10];
	volatile uint32_t C1IMR1;
	volatile uint32_t C1EMR1;
	volatile uint32_t C1PR1;
	uint32_t reserved5;
	volatile uint32_t C1IMR2;
	volatile uint32_t C1EMR2;
	volatile uint32_t C1PR2;
	uint32_t reserved6;
	volatile uint32_t C1IMR3;
	volatile uint32_t C1EMR3;
	volatile uint32_t C1PR3;
	uint32_t reserved2[5];
	volatile uint32_t C2IMR1;
	volatile uint32_t C2EMR1;
	volatile uint32_t C2PR1;
	uint32_t reserved7;
	volatile uint32_t C2IMR2;
	volatile uint32_t C2EMR2;
	volatile uint32_t C2PR2;
	uint32_t reserved8;
	volatile uint32_t C2IMR3;
	volatile uint32_t C2EMR3;
	volatile uint32_t C2PR3;
}EXTI_reg_t;

#define EXTI	((EXTI_reg_t *)EXTI_BASE_ADDR)

#endif /* STM32H7XX_HAL_INC_HAL_EXTI_H_ */
