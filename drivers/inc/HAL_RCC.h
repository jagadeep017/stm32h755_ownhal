/*
 * HAL_RCC.h
 *
 *  Created on: Oct 1, 2025
 *      Author: jagadeep.g
 */

#ifndef STM32H7XX_HAL_INC_HAL_RCC_H_
#define STM32H7XX_HAL_INC_HAL_RCC_H_

#include <HAL_BUS.h>
#include <stdint.h>

typedef struct{
	volatile uint32_t CR;			//0x0000
	volatile uint32_t HSICFGR;		//0x0004
	volatile uint32_t CRRCR;		//0x0008
	volatile uint32_t CSICFGR;		//0x000C
	volatile uint32_t CFGR;			//0x0010
	uint32_t reserved;				//0x0014
	volatile uint32_t D1CFGR;		//0x0018
	volatile uint32_t D2CFGR;		//0x001C
	volatile uint32_t D3CFGR;		//0x0020
	uint32_t reserved2;				//0x0024
	volatile uint32_t PLLCKSELR;	//0x0028
	volatile uint32_t PLLCFGR;		//0x002C
	volatile uint32_t PLL1DIVR;		//0x0030
	volatile uint32_t PLL1FRACR;	//0x0034
	volatile uint32_t PLL2DIVR;		//0x0038
	volatile uint32_t PLL2FRACR;	//0x003C
	volatile uint32_t PLL3DIVR;		//0x0040
	volatile uint32_t PLL3FRACR;	//0x0044
	uint32_t reserved3;				//0x0048
	volatile uint32_t D1CCIPR;		//0x004C
	volatile uint32_t D2CCIP1R;		//0x0050
	volatile uint32_t D2CCIP2R;		//0x0054
	volatile uint32_t D3CCIPR;		//0x0058
	uint32_t reserved4;				//0x005C
	volatile uint32_t CIER;			//0x0060
	volatile uint32_t CIFR;			//0x0064
	volatile uint32_t CICR;			//0x0068
	uint32_t reserved5;				//0x006C
	volatile uint32_t BDCR;			//0x0070
	volatile uint32_t CSR;			//0x0074
	uint32_t reserved6;				//0x0078
	volatile uint32_t AHB3RSTR;		//0x007C
	volatile uint32_t AHB1RSTR;		//0x0080
	volatile uint32_t AHB2RSTR;		//0x0084
	volatile uint32_t AHB4RSTR;		//0x0088
	volatile uint32_t APB3RSTR;		//0x008C
	volatile uint32_t APB1LRSTR;	//0x0090
	volatile uint32_t APB1HRSTR;	//0x0094
	volatile uint32_t APB2RSTR;		//0x0098
	volatile uint32_t APB4RSTR;		//0x009C
	volatile uint32_t GCR;			//0x00A0
	uint32_t reserved7;				//0x00A4
	volatile uint32_t D3AMR;		//0x00A8
	uint32_t reserved8[9];			//0x00AC-CC
	volatile uint32_t RSR;			//0x00D0
	volatile uint32_t AHB3ENR;		//0x00D4
	volatile uint32_t AHB1ENR;		//0x00D8
	volatile uint32_t AHB2ENR;		//0x00DC
	volatile uint32_t AHB4ENR;		//0x00E0
	volatile uint32_t APB3ENR;		//0x00E4
	volatile uint32_t APB1LENR;		//0x00E8
	volatile uint32_t APB1HENR;		//0x00EC
	volatile uint32_t APB2ENR;		//0x00F0
	volatile uint32_t APB4ENR;		//0x00F4
	uint32_t reserved9;				//0x00F8
	volatile uint32_t AHB3LPENR;	//0x00FC
	volatile uint32_t AHB1LPENR;	//0x0100
	volatile uint32_t AHB2LPENR;	//0x0104
	volatile uint32_t AHB4LPENR;	//0x0108
	volatile uint32_t APB3LPENR;	//0x010C
	volatile uint32_t APB1LLPENR;	//0x0110
	volatile uint32_t APB1HLPENR;	//0x0114
	volatile uint32_t APB2LPENR;	//0x0118
	volatile uint32_t APB4LPENR;	//0x011C
	uint32_t reserved10[4];			//0x120-0x12C
	volatile uint32_t C1_RSR;
	volatile uint32_t C1_AHB3ENR;
	volatile uint32_t C1_AHB1ENR;
	volatile uint32_t C1_AHB2ENR;
	volatile uint32_t C1_AHB4ENR;
	volatile uint32_t C1_APB3ENR;
	volatile uint32_t C1_APB1LENR;
	volatile uint32_t C1_APB1HENR;
	volatile uint32_t C1_APB2ENR;
	volatile uint32_t C1_APB4ENR;
	uint32_t reserved11;
	volatile uint32_t C1_AHB3LPENR;
	volatile uint32_t C1_AHB1LPENR;
	volatile uint32_t C1_AHB2LPENR;
	volatile uint32_t C1_AHB4LPENR;
	volatile uint32_t C1_APB3LPENR;
	volatile uint32_t C1_APB1LLPENR;
	volatile uint32_t C1_APB1HLPENR;
	volatile uint32_t C1_APB2LPENR;
	volatile uint32_t C1_APB4LPENR;
	uint32_t reserved12[4];
	volatile uint32_t C2_RSR;
	volatile uint32_t C2_AHB3ENR;
	volatile uint32_t C2_AHB1ENR;
	volatile uint32_t C2_AHB2ENR;
	volatile uint32_t C2_AHB4ENR;
	volatile uint32_t C2_APB3ENR;
	volatile uint32_t C2_APB1LENR;
	volatile uint32_t C2_APB1HENR;
	volatile uint32_t C2_APB2ENR;
	volatile uint32_t C2_APB4ENR;
	uint32_t reserved13;
	volatile uint32_t C2_AHB3LPENR;
	volatile uint32_t C2_AHB1LPENR;
	volatile uint32_t C2_AHB2LPENR;
	volatile uint32_t C2_AHB4LPENR;
	volatile uint32_t C2_APB3LPENR;
	volatile uint32_t C2_APB1LLPENR;
	volatile uint32_t C2_APB1HLPENR;
	volatile uint32_t C2_APB2LPENR;
	volatile uint32_t C2_APB4LPENR;
	uint32_t reserved14[8];
}RCC_reg_t;

#define RCC		((RCC_reg_t*)RCC_BASE_ADDR)


/*
 * Clock enable macro for GPIOx peripherals
 */
#define GPIOA_CLK_EN()		(RCC->AHB4ENR |= (1<<0))		//setting the 0th bit
#define GPIOB_CLK_EN()		(RCC->AHB4ENR |= (1<<1))		//setting the 1th bit
#define GPIOC_CLK_EN()		(RCC->AHB4ENR |= (1<<2))		//setting the 2th bit
#define GPIOD_CLK_EN()		(RCC->AHB4ENR |= (1<<3))		//setting the 3th bit
#define GPIOE_CLK_EN()		(RCC->AHB4ENR |= (1<<4))		//setting the 4th bit
#define GPIOF_CLK_EN()		(RCC->AHB4ENR |= (1<<5))		//setting the 5th bit
#define GPIOG_CLK_EN()		(RCC->AHB4ENR |= (1<<6))		//setting the 6th bit
#define GPIOH_CLK_EN()		(RCC->AHB4ENR |= (1<<7))		//setting the 7th bit
#define GPIOI_CLK_EN()		(RCC->AHB4ENR |= (1<<8))		//setting the 8th bit
#define GPIOJ_CLK_EN()		(RCC->AHB4ENR |= (1<<9))		//setting the 9th bit
#define GPIOK_CLK_EN()		(RCC->AHB4ENR |= (1<<10))		//setting the 10th bit


/*
 * Clock disable macro for GPIOx peripherals
 */
#define GPIOA_CLK_DI()		(RCC->AHB4ENR &= ~(1<<0))		//clearing the 0th bit
#define GPIOB_CLK_DI()		(RCC->AHB4ENR &= ~(1<<1))		//clearing the 1th bit
#define GPIOC_CLK_DI()		(RCC->AHB4ENR &= ~(1<<2))		//clearing the 2th bit
#define GPIOD_CLK_DI()		(RCC->AHB4ENR &= ~(1<<3))		//clearing the 3th bit
#define GPIOE_CLK_DI()		(RCC->AHB4ENR &= ~(1<<4))		//clearing the 4th bit
#define GPIOF_CLK_DI()		(RCC->AHB4ENR &= ~(1<<5))		//clearing the 5th bit
#define GPIOG_CLK_DI()		(RCC->AHB4ENR &= ~(1<<6))		//clearing the 6th bit
#define GPIOH_CLK_DI()		(RCC->AHB4ENR &= ~(1<<7))		//clearing the 7th bit
#define GPIOI_CLK_DI()		(RCC->AHB4ENR &= ~(1<<8))		//clearing the 8th bit
#define GPIOJ_CLK_DI()		(RCC->AHB4ENR &= ~(1<<9))		//clearing the 9th bit
#define GPIOK_CLK_DI()		(RCC->AHB4ENR &= ~(1<<10))		//clearing the 10th bit


/*
 * Clock enable and disable macro for SYSCFG
 */
#define SYSCFG_CLK_EN()		(RCC->APB4ENR |= (1<<1))
#define SYSCFG_CLK_DI()		(RCC->APB4ENR &= ~(1<<1))

/*
 * Clock enable and disable for serial communication (SPI, I2C, UART)
 */
#define SPI1_CLK_EN()		(RCC->APB2ENR |= (1<<12))
#define SPI1_CLK_DI()		(RCC->APB2ENR &= ~(1<<12))
#define SPI2_CLK_EN()		(RCC->APB1LENR |= (1<<14))
#define SPI2_CLK_DI()		(RCC->APB1LENR &= ~(1<<14))
#define SPI3_CLK_EN()		(RCC->APB1LENR |= (1<<15))
#define SPI3_CLK_DI()		(RCC->APB1LENR &= ~(1<<15))
#define SPI4_CLK_EN()		(RCC->APB2ENR |= (1<<13))
#define SPI4_CLK_DI()		(RCC->APB2ENR &= ~(1<<13))
#define SPI5_CLK_EN()		(RCC->APB2ENR |= (1<<20))
#define SPI5_CLK_DI()		(RCC->APB2ENR &= ~(1<<20))
#define SPI6_CLK_EN()		(RCC->APB4ENR |= (1<<5))
#define SPI6_CLK_DI()		(RCC->APB4ENR &= ~(1<<5))

#define I2C1_CLK_EN()		(RCC->APB1LENR |= (1<<21))
#define I2C1_CLK_DI()		(RCC->APB1LENR &= ~(1<<21))
#define I2C2_CLK_EN()		(RCC->APB1LENR |= (1<<22))
#define I2C2_CLK_DI()		(RCC->APB1LENR &= ~(1<<22))
#define I2C3_CLK_EN()		(RCC->APB1LENR |= (1<<23))
#define I2C3_CLK_DI()		(RCC->APB1LENR &= ~(1<<23))
#define I2C4_CLK_EN()		(RCC->APB4ENR |= (1<<7))
#define I2C4_CLK_DI()		(RCC->APB4ENR &= ~(1<<7))

#endif /* STM32H7XX_HAL_INC_HAL_RCC_H_ */
