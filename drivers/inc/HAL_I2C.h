/*
 * HAL_I2C.h
 *
 *  Created on: Oct 17, 2025
 *      Author: jagadeep.g
 */

#ifndef STM32H7XX_HAL_INC_HAL_I2C_H_
#define STM32H7XX_HAL_INC_HAL_I2C_H_

#include<stdint.h>
#include<HAL_BUS.h>

typedef struct{
	volatile uint32_t	CR1;
	volatile uint32_t	CR2;
	volatile uint32_t	OAR1;
	volatile uint32_t	OAR2;
	volatile uint32_t	TIMINGR;
	volatile uint32_t	TIMEOUTR;
	volatile uint32_t	ISR;
	volatile uint32_t	ICR;
	volatile uint32_t	PECR;
	volatile uint32_t	RXDR;
	volatile uint32_t	TXDR;
}I2C_reg_t;


typedef struct{
	uint32_t SCLtiming;			//@SCL_Timing
	uint16_t DeviceAddress1;	//own address1 7 or 10 bits
	uint8_t  Address1Mode;		//@I2C_ADDR_MODE
	uint8_t  DualAddressMode;	//enable or disable (enables address 2)
	uint8_t  DeviceAddress2;	//own address2 7bits
	uint8_t  Address2Mask;		//@I2C_MASK
	uint8_t  NoStretchMode;		//@I2C_STRETCH
}I2C_config_t;

typedef struct{
	I2C_reg_t    *I2Cx;
	I2C_config_t I2C_Config;
}I2C_handle_t;


#define I2C1		((I2C_reg_t*)I2C1_BASE_ADDR)
#define I2C2		((I2C_reg_t*)I2C2_BASE_ADDR)
#define I2C3		((I2C_reg_t*)I2C3_BASE_ADDR)
#define I2C4		((I2C_reg_t*)I2C4_BASE_ADDR)

#define I2C_CR1_PE_POS			0
#define I2C_CR1_TXIE_POS		1
#define I2C_CR1_RXIE_POS		2
#define I2C_CR1_ADDRIE_POS		3
#define I2C_CR1_NACKIE_POS		4
#define I2C_CR1_STOPIE_POS		5
#define I2C_CR1_TCIE_POS		6
#define I2C_CR1_ERRIE_POS		7
#define I2C_CR1_DNF_POS			8
#define I2C_CR1_ANFOFF_POS		12
#define I2C_CR1_TXDMAEN_POS		14
#define I2C_CR1_RXDMAEN_POS		15
#define I2C_CR1_SBC_POS			16
#define I2C_CR1_NOSTRETCH_POS	17
#define I2C_CR1_WUPEN_POS		18
#define I2C_CR1_GCEN_POS		19
#define I2C_CR1_SMBHEN_POS		20
#define I2C_CR1_SMBDEN_POS		21
#define I2C_CR1_ALERTEN_POS		22
#define I2C_CR1_PECEN_POS		23


#define I2C_CR2_SADD_POS		0
#define I2C_CR2_RDWRN_POS		10
#define I2C_CR2_ADD10_POS		11
#define I2C_CR2_HEAD10R_POS		12
#define I2C_CR2_START_POS		13
#define I2C_CR2_STOP_POS		14
#define I2C_CR2_NACK_POS		15
#define I2C_CR2_NBYTES_POS		16
#define I2C_CR2_RELOAD_POS		24
#define I2C_CR2_AUTOEND_POS		25
#define I2C_CR2_PECBYTE_POS		26


#define I2C_OAR1_OA1_POS		0
#define I2C_OAR1_OA1MODE_POS	10
#define I2C_OAR1_OA1EN_POS		15


#define I2C_OAR2_OA2_POS		1
#define I2C_OAR2_OA2MSK_POS		8
#define I2C_OAR2_OA2EN_POS		15


#define I2C_TIMINGR_SCLL_POS	0
#define I2C_TIMINGR_SCLH_POS	8
#define I2C_TIMINGR_SDADEL_POS	16
#define I2C_TIMINGR_SCLDEL_POS	20
#define I2C_TIMINGR_PRESC_POS	28


#define I2C_TIMOUTR_TIMEOUTA_POS	0
#define I2C_TIMOUTR_TIDLE_POS		12
#define I2C_TIMOUTR_TIMEOUTEN_POS	15
#define I2C_TIMOUTR_TIMEOUTB_POS	16
#define I2C_TIMOUTR_TEXTEN_POS		31


#define I2C_ISR_TXE_POS			0
#define I2C_ISR_TXIS_POS		1
#define I2C_ISR_RXNE_POS		2
#define I2C_ISR_ADDR_POS		3
#define I2C_ISR_NACKF_POS		4
#define I2C_ISR_STOPF_POS		5
#define I2C_ISR_TC_POS			6
#define I2C_ISR_TCR_POS			7
#define I2C_ISR_BERR_POS		8
#define I2C_ISR_ARLO_POS		9
#define I2C_ISR_OVR_POS			10
#define I2C_ISR_PECERR_POS		11
#define I2C_ISR_TIMEOUT_POS		12
#define I2C_ISR_ALERT_POS		13
#define I2C_ISR_BUSY_POS		15
#define I2C_ISR_DIR_POS			16
#define I2C_ISR_ADDCODE_POS		17

#define I2C_FLAG_TXE                    (1<<I2C_ISR_TXE_POS)
#define I2C_FLAG_TXIS                   (1<<I2C_ISR_TXIS_POS)
#define I2C_FLAG_RXNE                   (1<<I2C_ISR_RXNE_POS)
#define I2C_FLAG_ADDR                   (1<<I2C_ISR_ADDR_POS)
#define I2C_FLAG_AF                     (1<<I2C_ISR_NACKF_POS)
#define I2C_FLAG_STOPF                  (1<<I2C_ISR_STOPF_POS)
#define I2C_FLAG_TC                     (1<<I2C_ISR_TC_POS)
#define I2C_FLAG_TCR                    (1<<I2C_ISR_TCR_POS)
#define I2C_FLAG_BERR                   (1<<I2C_ISR_BERR_POS)
#define I2C_FLAG_ARLO                   (1<<I2C_ISR_ARLO_POS)
#define I2C_FLAG_OVR                    (1<<I2C_ISR_OVR_POS)
#define I2C_FLAG_PECERR                 (1<<I2C_ISR_PECERR_POS)
#define I2C_FLAG_TIMEOUT                (1<<I2C_ISR_TIMEOUT_POS)
#define I2C_FLAG_ALERT                  (1<<I2C_ISR_ALERT_POS)
#define I2C_FLAG_BUSY                   (1<<I2C_ISR_BUSY_POS)
#define I2C_FLAG_DIR                    (1<<I2C_ISR_DIR_POS)
#define I2C_FLAG_ADDCODE				(0x7f<<I2C_ISR_ADDCODE_POS)

#define I2C_ICR_ADDRCF_POS			3
#define I2C_ICR_NACKCF_POS			4
#define I2C_ICR_STOPCF_POS			5
#define I2C_ICR_BERRCF_POS			8
#define I2C_ICR_ARLOCF_POS			9
#define I2C_ICR_OVRCF_POS			10
#define I2C_ICR_PECCF_POS			11
#define I2C_ICR_TIMEOUTCF_POS		12
#define I2C_ICR_ALERTCF_POS			13


#define I2C_PECR_PEC_POS			0


#define I2C_RXDR_RXDATA_POS			0


#define I2C_TXDR_TXDATA_POS			0


#define I2C1_REG_RESET()	do{(RCC->APB1LRSTR |= (1<<21));	(RCC->APB1LRSTR &= ~(1<<21));}while(0)
#define I2C2_REG_RESET()	do{(RCC->APB1LRSTR |= (1<<22));	(RCC->APB1LRSTR &= ~(1<<22));}while(0)
#define I2C3_REG_RESET()	do{(RCC->APB1LRSTR |= (1<<23));	(RCC->APB1LRSTR &= ~(1<<23));}while(0)
#define I2C4_REG_RESET()	do{(RCC->APB4RSTR |= (1<<7));	(RCC->APB4RSTR &= ~(1<<7));}while(0)

/*
 * @I2C_MASK
 */
#define I2C_OA2_NOMASK                  (0x00U)
#define I2C_OA2_MASK01                  (0x01U)
#define I2C_OA2_MASK02                  (0x02U)
#define I2C_OA2_MASK03                  (0x03U)
#define I2C_OA2_MASK04                  (0x04U)
#define I2C_OA2_MASK05                  (0x05U)
#define I2C_OA2_MASK06                  (0x06U)
#define I2C_OA2_MASK07                  (0x07U)


/*
 * @SCL_Timing
 */


/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE				1
#define I2C_ACK_DISABLE				0


/*
 * @I2C123_CLK_SEL
 */
#define I2C123_PCLK1_CK					0
#define I2C123_PLL3_CK					1
#define I2C123_HSI_CK					2
#define I2C123_CSI_CK					3

/*
 * @I2C4_CLK_SEL
 */
#define I2C4_PCLK4_CK					0
#define I2C4_PLL3_CK					1
#define I2C4_HSI_CK						2
#define I2C4_CSI_CK						3

/*
 * @I2C_ADDR_MODE
 */
#define I2C_7BIT_MODE				0
#define I2C_10BIT_MODE				1

#define I2C_DUAL_MODE				1
#define I2C_NO_DUAL_MODE			0

/*
 * @I2C_STRETCH
 */
#define I2C_NOSTRECH_DISABLE		0
#define I2C_NOSTRECH_ENSABLE		1


void HAL_I2C_CLK_control(I2C_handle_t *pHandle, uint8_t type);

void I2C123_CLK_SEL(uint8_t type);

void I2C4_CLK_SEL(uint8_t type);

void HAL_I2C_init(I2C_handle_t *pHandle);

void HAL_I2C_deinit(I2C_handle_t *pHandle);

void HAL_I2C_Master_Transmit(I2C_handle_t *pHandle, uint16_t Address, uint8_t *pData,
                                          uint16_t Size);
void HAL_I2C_Master_Receive(I2C_handle_t *pHandle, uint16_t Address, uint8_t *pData,
                                         uint16_t Size);

#endif /* STM32H7XX_HAL_INC_HAL_I2C_H_ */
