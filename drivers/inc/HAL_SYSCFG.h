/*
 * HAL_SYSCFG.h
 *
 *  Created on: Oct 6, 2025
 *      Author: jagadeep.g
 */

#ifndef STM32H7XX_HAL_INC_HAL_SYSCFG_H_
#define STM32H7XX_HAL_INC_HAL_SYSCFG_H_

#include <HAL_BUS.h>
#include <stdint.h>

typedef struct{
	uint32_t reserved;
	volatile uint32_t PMCR;
	volatile uint32_t EXTICR[4];
	volatile uint32_t CFGR;
	uint32_t reserved5;
	volatile uint32_t CCSR;
	volatile uint32_t CCVR;
	volatile uint32_t CCCR;
	volatile uint32_t PWRCR;
	uint32_t reserved2[52];
	volatile uint32_t SR0;
	uint32_t reserved3[8];
	volatile uint32_t PKGR;
	uint32_t reserved4[118];
	volatile uint32_t UR0;
	volatile uint32_t UR1;
	volatile uint32_t UR2;
	volatile uint32_t UR3;
	volatile uint32_t UR4;
	volatile uint32_t UR5;
	volatile uint32_t UR6;
	volatile uint32_t UR7;
	volatile uint32_t UR8;
	volatile uint32_t UR9;
	volatile uint32_t UR10;
	volatile uint32_t UR11;
	volatile uint32_t UR12;
	volatile uint32_t UR13;
	volatile uint32_t UR14;
	volatile uint32_t UR15;
	volatile uint32_t UR16;
	volatile uint32_t UR17;
}SYSCFG_reg_t;

#define SYSCFG		((SYSCFG_reg_t*) SYSCFG_BASE_ADDR)

#endif /* STM32H7XX_HAL_INC_HAL_SYSCFG_H_ */
