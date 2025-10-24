/*
 * HAL_IRQ.c
 *
 *  Created on: Oct 17, 2025
 *      Author: jagadeep.g
 */

#include <HAL_IRQ.h>
#include <HAL_BUS.h>

void HAL_irq_config(uint8_t irq_number, uint8_t type){
	//TODO need to check if correct parameters were passed
	uint8_t ind = irq_number/32			//index of ISER or ICER
			,pos = irq_number%32;		//position of that bit in ISER or ICER
	if(type == ENABLE){
		NVIC_ISER[ind] |= (1<<pos);
	}
	else{
		NVIC_ICER[ind] |= (1<<pos);
	}
}

void HAL_irq_priority(uint8_t irq_number, uint8_t priority){
	uint8_t ind = irq_number/4			//index of the register in IPR
			,pos = irq_number%4;		//position of bits in IPR
	//setting the priority
	NVIC_IPR[ind] |= ((uint32_t)priority<<(8*pos+4)); 		//because only 4 msb's are implemented in stm32 IPR
}
