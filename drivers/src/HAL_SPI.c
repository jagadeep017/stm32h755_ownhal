/*
 * HAL_SPI.c
 *
 *  Created on: Oct 7, 2025
 *      Author: jagadeep.g
 */

#include <HAL_SPI.h>
#include <HAL_RCC.h>
#include <HAL_IRQ.h>

/*
 * SPI init and deinit
 * SPIx		 :-	base address of SPI registers (SPI1-SPI6)
 * spi_config:-	parameters for config
 */
void HAL_SPI_init(SPI_handle_t *pSPI_handle){
	uint32_t temp = 0;
	//Disable the SPI peripheral by clearing SPE bit
	SPI_Peripheral_Control(pSPI_handle->SPIx,DISABLE);

	if(pSPI_handle->SPI_config.SSM == SPI_SSM_SW){
		pSPI_handle->SPIx->CR1 |= 1<<SPI_CR1_SSI_POS;
		//setting SSM
		pSPI_handle->SPIx->CFG2 |= (pSPI_handle->SPI_config.SSM<<SPI_CFG2_SSM_POS);
	}
	else{
		pSPI_handle->SPIx->CFG2 &= (1<<SPI_CFG2_SSM_POS);
		pSPI_handle->SPIx->CFG2 |= (1<<SPI_CFG2_SSOE_POS);
	}

	//setting CFG1 REG in temp buffer
	//setting CLOCK speed
	temp |= (pSPI_handle->SPI_config.SclkSpeed<<SPI_CFG1_MBR_POS);

	//setting data frame size
	temp |= (pSPI_handle->SPI_config.DFF);

	//	temp |= 0x3<<5;		//fthlv
	pSPI_handle->SPIx->CFG1 = temp;

	temp=0;		//clearing buffer for CFG2

	//master or slave
	if(pSPI_handle->SPI_config.DeviceMode == SPI_MODE_MASTER){
		pSPI_handle->SPIx->CFG2 |= (1<<SPI_CFG2_MASTER_POS);
	}
	else{
		pSPI_handle->SPIx->CFG2 &= ~(1<<SPI_CFG2_MASTER_POS);
	}

	//configuring SPI type in CFG2 buffer(temp)
	//setting bus type
	temp |= (pSPI_handle->SPI_config.BusConfig<<SPI_CFG2_COMM_POS);

	//setting CPHA and CPOL
	temp |= (pSPI_handle->SPI_config.CPHA<<SPI_CFG2_CPHA_POS);
	temp |= (pSPI_handle->SPI_config.CPOL<<SPI_CFG2_CPOL_POS);

	pSPI_handle->SPIx->CFG2 |= temp;

}

void HAL_SPI_deinit(SPI_reg_t *SPIx){
	if(SPIx == SPI1){
		SPI1_REG_RESET();
	}
	else if(SPIx == SPI2){
		SPI2_REG_RESET();
	}
	else if(SPIx == SPI3){
		SPI3_REG_RESET();
	}
	else if(SPIx == SPI4){
		SPI4_REG_RESET();
	}
	else if(SPIx == SPI5){
		SPI5_REG_RESET();
	}
	else if(SPIx == SPI6){
		SPI6_REG_RESET();
	}
}

/*
 * SPI Clock setup
 * SPIx	:-	base address of SPI registers (SPI1-SPI6)
 * type :-  ENABLE or DISABLE
 */
void HAL_SPI_CLK_control(SPI_reg_t *SPIx, uint8_t type){
	if(type == ENABLE){
		if(SPIx == SPI1){
			SPI1_CLK_EN();
		}
		else if(SPIx == SPI2){
			SPI2_CLK_EN();
		}
		else if(SPIx == SPI3){
			SPI3_CLK_EN();
		}
		else if(SPIx == SPI4){
			SPI4_CLK_EN();
		}
		else if(SPIx == SPI5){
			SPI5_CLK_EN();
		}
		else if(SPIx == SPI6){
			SPI6_CLK_EN();
		}
	}
	else{
		if(SPIx == SPI1){
			SPI1_CLK_DI();
		}
		else if(SPIx == SPI2){
			SPI2_CLK_DI();
		}
		else if(SPIx == SPI3){
			SPI3_CLK_DI();
		}
		else if(SPIx == SPI4){
			SPI4_CLK_DI();
		}
		else if(SPIx == SPI5){
			SPI5_CLK_DI();
		}
		else if(SPIx == SPI6){
			SPI6_CLK_DI();
		}
	}
}

/*
 * Data send and receive
 * len	:- 	length of buffer in bytes
 */
void SPI_send(SPI_reg_t *SPIx, uint8_t *pTxbuffer, uint16_t len){

	//setting comm in simplex stransmit
	SPIx->CFG2 &=~(0x3<<SPI_CFG2_COMM_POS);
	SPIx->CFG2 |= (0x1<<SPI_CFG2_COMM_POS);

	//load the data in tx data register based on the size
	uint8_t dff = SPIx->CFG1 & 0x1f;
	SPI_Peripheral_Control(SPIx, ENABLE);
	SPIx->CR2 |= len<<SPI_CR2_TSIZE_POS;


	SPIx->CR1 |= 1<<SPI_CR1_CSTART_POS;
	while(len>0){
		while(!(SPIx->SR & 1<<SPI_SR_TXP_POS));	//check whether there is space for 1 next data packet in TxFIFO

		if(dff == SPI_DFF_8BITS){			//size is 8bits
			*((uint8_t*)(&SPIx->TXDR)) = *pTxbuffer;
			pTxbuffer++;
			len--;
		}
		else if(dff == SPI_DFF_16BITS){		//size is 16bits
			*((uint16_t*)(&SPIx->TXDR)) = *((uint16_t*)pTxbuffer);
			pTxbuffer+=2;
			len-=2;
		}
		else if(dff == SPI_DFF_32BITS){		//size is 32bits
			*((uint32_t*)(&SPIx->TXDR)) = *((uint32_t*)pTxbuffer);
			pTxbuffer+=4;
			len-=4;
		}
		else{
			SPI_Peripheral_Control(SPIx, DISABLE);
			return;
		}
	}

	while(!(SPIx->SR&SPI_SR_TXC_POS));		//wait till all the data in txFIFO is transmitted
	SPI_Peripheral_Control(SPIx, DISABLE);
	SPIx->CR2 &= ~(0xffff<<SPI_CR2_TSIZE_POS);
	SPIx->IFCR |= 1<<4;		//clearing txtf flag

}

void SPI_recv(SPI_reg_t *SPIx, uint8_t *pRxbuffer, uint32_t len){
	//read the data from rx data register based on the size
	//setting comm in simplex stransmit
	SPIx->CFG2 &=~(0x3<<SPI_CFG2_COMM_POS);
	SPIx->CFG2 |= (0x2<<SPI_CFG2_COMM_POS);
	uint8_t dff = SPIx->CFG1 & 0x1f;
	SPI_Peripheral_Control(SPIx, ENABLE);
	SPIx->CR1 |= 1<<SPI_CR1_CSTART_POS;
	while(len>0){
		while(!(SPIx->SR & 1<<SPI_SR_RXP_POS));

		if(dff == SPI_DFF_8BITS){			//size is 8bits
			*(pRxbuffer) = *(uint8_t*)(&SPIx->RXDR);
			pRxbuffer++;
			len--;
		}
		else if(dff == SPI_DFF_16BITS){		//size is 16bits
			*((uint16_t*)pRxbuffer) = *(uint16_t*)(&SPIx->RXDR);
			pRxbuffer+=2;
			len-=2;
		}
		else if(dff == SPI_DFF_32BITS){		//size is 32bits
			*((uint32_t*)pRxbuffer) = *(uint32_t*)(&SPIx->RXDR);
			pRxbuffer+=4;
			len-=4;
		}
		else{
			break;
		}
	}
	SPI_Peripheral_Control(SPIx, DISABLE);
}



/*
 *  API to ENABLE or DISABLE SPIx peripheral
 */
void SPI_Peripheral_Control(SPI_reg_t *SPIx, uint8_t type){
	if(type == ENABLE){
		SPIx->CR1 |= (1<<SPI_CR1_SPE_POS);
	}
	else{
		SPIx->CR1 &= ~(1<<SPI_CR1_SPE_POS);
	}
}

/*
 * SPI123 Clock Select
 * type :- SPI123_PLL1_CK,SPI123_PLL2_CK,SPI123_PLL3_CK , SPI123_I2S_CK, SPI123_PER_CK
 */
void SPI123_CLK_SEL(uint8_t type){
	RCC->D2CCIP1R |= type <<D2CCIP1R_SPI123SEL_POS;
}

/*
 * SPI45 Clock Select
 * type :- SPI45_APB_CK,SPI45_PLL2_CK,SPI45_PLL3_CK , SPI45_HSI_CK, SPI45_CSI_CK,SPI45_HSE_CK
 */
void SPI45_CLK_SEL(uint8_t type){
	RCC->D2CCIP1R |= type <<D2CCIP1R_SPI45SEL_POS;
}

/*
 * SPI45 Clock Select
 * type :- SPI6_RCC_CK, SPI6_PLL2_CK, SPI6_PLL3_CK, SPI6_HSI_CK, SPI6_CSI_CK, SPI6_HSE_CK
 *  */
void SPI6_CLK_SEL(uint8_t type){
	RCC->D3CCIPR |= type <<D3CCIPR_SPI6SEL_POS;
}

uint8_t SPI_send_INT(SPI_handle_t *pSPIhandle, uint8_t *pTxBuffer, uint16_t Len){

	//setting length in tsize
	pSPIhandle->SPIx->CR2 |= Len<<SPI_CR2_TSIZE_POS;
	pSPIhandle->SPIx->CFG2 &=~(0x3<<SPI_CFG2_COMM_POS);
	pSPIhandle->SPIx->CFG2 |= (0x1<<SPI_CFG2_COMM_POS);
	SPI_Peripheral_Control(pSPIhandle->SPIx, ENABLE);
	if(pSPIhandle->TxState != SPI_TX_INT_BUSY){
		//setting the buffer and its length
		pSPIhandle->pTxBuffer = pTxBuffer;
		pSPIhandle->TxLen = Len;

		//setting the busy flag
		pSPIhandle->TxState = SPI_TX_INT_BUSY;


		pSPIhandle->SPIx->CR1 |= 1<<SPI_CR1_CSTART_POS;

		//setting the TXEIE control bit to get interrupt when TXE flag is set in SR
		pSPIhandle->SPIx->IER |= (1<<SPI_IER_TXPIE_POS);
		//data is transmission will happen in IRQ code
		return SPI_READY;
	}
	return SPI_TX_INT_BUSY;
}


uint8_t SPI_recv_INT(SPI_handle_t *pSPIhandle, uint8_t *pRxBuffer, uint32_t Len){
	if(pSPIhandle->RxState != SPI_RX_INT_BUSY){
		pSPIhandle->SPIx->CR2 |= Len<<SPI_CR2_TSIZE_POS;
		pSPIhandle->SPIx->CFG2 &=~(0x3<<SPI_CFG2_COMM_POS);
		pSPIhandle->SPIx->CFG2 |= (0x2<<SPI_CFG2_COMM_POS);
		SPI_Peripheral_Control(pSPIhandle->SPIx, ENABLE);
		//setting the buffer and its length
		pSPIhandle->pRxBuffer = pRxBuffer;
		pSPIhandle->RxLen = Len;

		//setting the busy flag
		pSPIhandle->RxState = SPI_RX_INT_BUSY;

		pSPIhandle->SPIx->CR1 |= 1<<SPI_CR1_CSTART_POS;
		//data is transmission will happen in IRQ code

		//setting the TXEIE control bit to get interrupt when TXE flag is set in SR
		pSPIhandle->SPIx->IER |= (1<<SPI_IER_RXPIE_POS);


		return SPI_READY;
	}
	return SPI_RX_INT_BUSY;
}


void SPI_IRQ_handling(SPI_handle_t *pHandle){
	if((pHandle->SPIx->IER & (1<<SPI_IER_TXPIE_POS))&(pHandle->SPIx->SR &(1<<SPI_SR_TXP_POS))){			//transmit
		if(!(pHandle->SPIx->SR&(0xffff<<SPI_SR_CSTART_POS))){
			return;
		}
		uint8_t dff = pHandle->SPIx->CFG1 & (0x1f<<SPI_CFG1_DFF_POS);
		if(dff == SPI_DFF_8BITS){			//size is 8bits
			*((uint8_t*)(&pHandle->SPIx->TXDR)) = *pHandle->pTxBuffer;
			pHandle->pTxBuffer++;
			pHandle->TxLen--;
		}
		else if(dff == SPI_DFF_16BITS){		//size is 16bits
			*((uint16_t*)(&pHandle->SPIx->TXDR)) = *((uint16_t*)pHandle->pTxBuffer);
			pHandle->pTxBuffer+=2;
			pHandle->TxLen-=2;
		}
		else if(dff == SPI_DFF_32BITS){		//size is 32bits
			*((uint32_t*)(&pHandle->SPIx->TXDR)) = *((uint32_t*)pHandle->pTxBuffer);
			pHandle->pTxBuffer+=4;
			pHandle->TxLen-=4;
		}
		else{
			return;
		}
		if(pHandle->TxLen <= 0){
			//close the transmission
			SPI_Close_Tx(pHandle);
			SPI_Appliaction_callback(pHandle, SPI_EVENT_TX_CMPLT);
		}
	}
	if((pHandle->SPIx->IER & (1<<SPI_IER_RXPIE_POS)&(pHandle->SPIx->SR &(1<<SPI_SR_RXP_POS)))){		//receive
		uint8_t dff = pHandle->SPIx->CFG1 & (0x1f<<SPI_CFG1_DFF_POS);
		if(dff == SPI_DFF_8BITS){			//size is 8bits
			*(pHandle->pRxBuffer) = *((uint8_t*)(&pHandle->SPIx->RXDR));
			pHandle->pRxBuffer++;
			pHandle->RxLen--;
		}
		else if(dff == SPI_DFF_16BITS){		//size is 16bits
			*((uint16_t*)pHandle->pRxBuffer) = *((uint16_t*)(&pHandle->SPIx->RXDR));
			pHandle->pRxBuffer+=2;
			pHandle->RxLen-=2;
		}
		else if(dff == SPI_DFF_32BITS){		//size is 32bits
			*((uint32_t*)pHandle->pRxBuffer) = *((uint32_t*)(&pHandle->SPIx->RXDR));
			pHandle->pRxBuffer+=4;
			pHandle->RxLen-=4;
		}
		else{
			return;
		}
		if(pHandle->RxLen == 0){
			//close the transmission
			SPI_Close_Rx(pHandle);
			SPI_Appliaction_callback(pHandle, SPI_EVENT_RX_CMPLT);
		}
	}
	if((pHandle->SPIx->IER & (1<<SPI_IER_OVRIE_POS)&(pHandle->SPIx->SR &(1<<SPI_SR_OVR_POS)))){	//overrun INTERRUPT
		//clearing the ovrflag
		pHandle->SPIx->IFCR &= ~(1<<SPI_IFCR_OVRC_POS);
		//inform the application
		SPI_Appliaction_callback(pHandle, SPI_EVENT_OVR_ERR);
	}
}

__attribute__((weak)) void SPI_Appliaction_callback(SPI_handle_t *Handle, uint8_t event){


}


void SPI_Clear_OVR(SPI_reg_t *SPIx){

}

void SPI_Close_Tx(SPI_handle_t *pHandle){
	pHandle->SPIx->IER &= ~(1<<SPI_IER_TXPIE_POS);
	pHandle->TxLen = 0;
	pHandle->pTxBuffer = (void *)0;
	pHandle->TxState = SPI_READY;
	pHandle->SPIx->CR2 &= ~(0xffff<<SPI_CR2_TSIZE_POS);	//clearing tsize after transmission
	pHandle->SPIx->IFCR |= 1<<4;
	SPI_Peripheral_Control(pHandle->SPIx, DISABLE);
}

void SPI_Close_Rx(SPI_handle_t *pHandle){
	pHandle->SPIx->IER &= ~(1<<SPI_IER_RXPIE_POS);
	pHandle->RxLen = 0;
	pHandle->pRxBuffer = (void *)0;
	pHandle->RxState = SPI_READY;
	pHandle->SPIx->CR2 &= ~(0xffff<<SPI_CR2_TSIZE_POS);	//clearing tsize after reception
	SPI_Peripheral_Control(pHandle->SPIx, DISABLE);
}
