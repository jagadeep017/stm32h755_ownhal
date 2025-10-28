/*
 * HAL_I2C.c
 *
 *  Created on: Oct 17, 2025
 *      Author: jagadeep.g
 */


#include <HAL_I2C.h>
#include <HAL_RCC.h>
#include <HAL_IRQ.h>

static void I2C_Generate_Start(I2C_reg_t *pI2Cx);

void I2C_Peripheral_Control(I2C_reg_t *pI2Cx,uint8_t type){
	if(type == ENABLE){
		pI2Cx->CR1 |= 1<<I2C_CR1_PE_POS;
	}
	else{
		pI2Cx->CR1 &= ~(1<<I2C_CR1_PE_POS);
	}
}



void HAL_I2C_CLK_control(I2C_handle_t *pHandle, uint8_t type){
	if(type == ENABLE){
		if(pHandle->I2Cx == I2C1){
			I2C1_CLK_EN();
		}
		else if(pHandle->I2Cx == I2C2){
			I2C2_CLK_EN();
		}
		else if(pHandle->I2Cx == I2C3){
			I2C3_CLK_EN();
		}
		else if(pHandle->I2Cx == I2C4){
			I2C4_CLK_EN();
		}
	}
	else{
		if(pHandle->I2Cx == I2C1){
			I2C1_CLK_DI();
		}
		else if(pHandle->I2Cx == I2C2){
			I2C2_CLK_DI();
		}
		else if(pHandle->I2Cx == I2C3){
			I2C3_CLK_DI();
		}
		else if(pHandle->I2Cx == I2C4){
			I2C4_CLK_DI();
		}
	}

}

/*
 * type = @I2C123_CLK_SEL
 */
void I2C123_CLK_SEL(uint8_t type){
	RCC->D2CCIP2R &= ~(0x3<<12);
	RCC->D2CCIP2R |= (type<<12);
}

/*
 * type = @I2C4_CLK_SEL
 */
void I2C4_CLK_SEL(uint8_t type){
	RCC->D3CCIPR &= ~(0x3<<8);
	RCC->D3CCIPR |= type<<8;
}


void HAL_I2C_init(I2C_handle_t *pHandle){
	//disable i2c peripheral
	I2C_Peripheral_Control(pHandle->I2Cx, DISABLE);
	//todo configure ANFOFF and DNF in CR1

	//todo need to calculate i2c clock and based on the calculate for TIMINGR register
	pHandle->I2Cx->TIMINGR = pHandle->I2C_Config.SCLtiming;


	//disable oar1 enable and clearing previous config
	pHandle->I2Cx->OAR1 = 0;
	//setting 7 bit or 10bit mode
	pHandle->I2Cx->OAR1 |= pHandle->I2C_Config.Address1Mode;
	if(pHandle->I2C_Config.Address1Mode == I2C_7BIT_MODE){
		pHandle->I2Cx->OAR1 |= pHandle->I2C_Config.DeviceAddress1<<1;
		pHandle->I2Cx->CR2 &= ~(1<<I2C_CR2_ADD10_POS);
	}
	else{
		pHandle->I2Cx->OAR1 |= pHandle->I2C_Config.DeviceAddress1;
		pHandle->I2Cx->CR2 |= 1<<I2C_CR2_ADD10_POS;
	}
	//enabling address 1
	pHandle->I2Cx->OAR1 |= 1<<I2C_OAR1_OA1EN_POS;
	//disable oar2 enable and clearing previous config
	pHandle->I2Cx->OAR2 = 0;
	if(pHandle->I2C_Config.DualAddressMode == I2C_DUAL_MODE){
		pHandle->I2Cx->OAR2 |= (pHandle->I2C_Config.DeviceAddress2<<I2C_OAR2_OA2_POS) | (1<<I2C_OAR2_OA2EN_POS) | (pHandle->I2C_Config.Address2Mask<<I2C_OAR2_OA2MSK_POS);
	}

	//configure no stretch in CR1
	pHandle->I2Cx->CR1 |= pHandle->I2C_Config.NoStretchMode<<I2C_CR1_NOSTRETCH_POS;

	//enabling the peripheral
	I2C_Peripheral_Control(pHandle->I2Cx, ENABLE);
}



void HAL_I2C_deinit(I2C_handle_t *pHandle){
	if(pHandle->I2Cx == I2C1){
		I2C1_REG_RESET();
	}
	else if(pHandle->I2Cx == I2C2){
		I2C2_REG_RESET();
	}
	else if(pHandle->I2Cx == I2C3){
		I2C3_REG_RESET();
	}
	else if(pHandle->I2Cx == I2C4){
		I2C4_REG_RESET();
	}
}

void HAL_I2C_Master_Transmit(I2C_handle_t *pHandle, uint16_t Address, uint8_t *pData, uint16_t Size){

	//configuring slave address
	if(Address>0x7F){
		pHandle->I2Cx->CR2 |= 1<<I2C_CR2_ADD10_POS;
		pHandle->I2Cx->CR2 |= (Address<<I2C_CR2_SADD_POS);
	}
	else{
		pHandle->I2Cx->CR2 &= ~(1<<I2C_CR2_ADD10_POS);
		pHandle->I2Cx->CR2 |= ((Address<<I2C_CR2_SADD_POS)<<1);
	}
	if(Size>255){

	}
	else{
		//set Size in NBYTES
		pHandle->I2Cx->CR2 |= Size<<I2C_CR2_NBYTES_POS;
		pHandle->I2Cx->CR2 |= 1<<I2C_CR2_AUTOEND_POS;	//setting autoend
		pHandle->I2Cx->CR2 &= ~(1<<I2C_CR2_RELOAD_POS);	//clearing reload

		I2C_Generate_Start(pHandle->I2Cx);

		while(Size){
			while((!(pHandle->I2Cx->ISR & (1<<I2C_ISR_TXIS_POS)))&(!(pHandle->I2Cx->ISR & (1<<I2C_ISR_NACKF_POS))));
			if(pHandle->I2Cx->ISR & (1<<I2C_ISR_NACKF_POS)){
				return;
			}
			pHandle->I2Cx->TXDR = *pData;
			pData++;
			Size--;
		}

		if(pHandle->I2Cx->ISR & (1<<I2C_ISR_TC_POS)){

		}

	}
}
void HAL_I2C_Master_Receive(I2C_handle_t *pHandle, uint16_t Address, uint8_t *pData, uint16_t Size){

}

static void I2C_Generate_Start(I2C_reg_t *pI2Cx){
	pI2Cx->CR2 |= 1<<I2C_CR2_START_POS;
}
