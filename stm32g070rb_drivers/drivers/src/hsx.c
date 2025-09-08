/*
 * hsx.c
 *
 *  Created on: Sep 8, 2025
 *      Author: kaush
 */

#include <stdint.h>

#define RCC_BASE_ADDR		0x40021000UL

#define RCC_CFGR_OFFSET 	0X08UL
#define RCC_CFGR_ADDR		(RCC_BASE_ADDR + RCC_CFGR_OFFSET)

#define RCC_IOPENR_OFFSET	0x34UL
#define RCC_IOPENR_ADDR		(RCC_BASE_ADDR + RCC_IOPENR_OFFSET)

#define GPIOA_BASE_ADDR		0x50000000UL

#define GPIOx_MODE_OFFSET	0x00UL
#define GPIOx_MODE_ADDR		(GPIOA_BASE_ADDR + GPIOx_MODE_OFFSET)

#define GPIOx_AFHR_OFFSET	0x24UL
#define GPIOx_AFHR_ADDR		(GPIOA_BASE_ADDR + GPIOx_AFHR_OFFSET)

void hsi_mco_init(void) {
	volatile uint32_t *pRccCfgrReg = (volatile uint32_t *)(RCC_CFGR_ADDR);

	// Configure the MCO2SEL bit to configure the MCO Signal
	*pRccCfgrReg &= ~(0x0F << 24);
	*pRccCfgrReg |= (0x03 << 24);

	// Configure the Pre-Scaler value (1024)
	*pRccCfgrReg &= ~(0x0F << 28);
	*pRccCfgrReg |= (0x02 << 28);

	volatile uint32_t *pRccIopenrReg = (volatile uint32_t *)(RCC_IOPENR_ADDR);

	// Enable the Clock for GPIOA
	*pRccIopenrReg |= (0x01 << 0);

	volatile uint32_t *pGpioxModeReg = (volatile uint32_t *)(GPIOx_MODE_ADDR);

	// Configure the Mode for PA8
	*pGpioxModeReg &= ~(0x03 << 16);
	*pGpioxModeReg |= (0x02 << 16);

	volatile uint32_t *pGpioxAfrhReg = (volatile uint32_t *)(GPIOx_AFHR_ADDR);


	// Configure the Alternate functionality of PA8
	*pGpioxAfrhReg &= ~(0x0F << 0);
}

