/*
 * stm32g070_gpio_driver.c
 *
 *  Created on: Sep 4, 2025
 *      Author: Koushik Bhat
 */

#include "stm32g070_gpio_driver.h"

/*
 * Peripheral Clock Setup
 */

void GPIO_PeriphClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enable) {
    if (enable) {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_EN();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_EN();
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_EN();
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_EN();
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_EN();
        } else if (pGPIOx == GPIOF) {
            GPIOF_PCLK_EN();
        }
    } else {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_DS();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_DS();
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_DS();
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_DS();
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_DS();
        } else if (pGPIOx == GPIOF) {
            GPIOF_PCLK_DS();
        }
    }
}

/*
 * Initialization and De-Initialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint8_t regValue = 0x00;

	// Pin Configuration
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {	// Non-Interrupt Mode
		regValue = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->GPIOx_MODER &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->GPIOx_MODER |= regValue;
	} else {
		// Interrupt Mode
	}

	// Output Type Configuration
	regValue = 0x00;
	regValue = pGPIOHandle->GPIO_PinConfig.GPIO_OutputType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->GPIOx_OTYPER &= ~(0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->GPIOx_OTYPER |= regValue;

	// Output Speed Configuration
	regValue = 0x00;
	regValue = pGPIOHandle->GPIO_PinConfig.GPIO_OutputSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->GPIOx_OSPEEDR &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->GPIOx_OTYPER |= regValue;

	// PullUP-PullDown Configuration
	regValue = 0x00;
	regValue = pGPIOHandle->GPIO_PinConfig.GPIO_PullUpPullDownConfig << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->GPIOx_PUPDR &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->GPIOx_PUPDR |= regValue;

	// Alternate Functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT_MODE) {
		regValue = 0x00;
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7) {
			regValue = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->GPIOx_AFRL &= ~(0x08 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->GPIOx_AFRL |= regValue;
		} else {
			uint8_t pinOffset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
			regValue = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pinOffset);
			pGPIOHandle->pGPIOx->GPIOx_AFRH &= ~(0x08 << (4 * pinOffset));
			pGPIOHandle->pGPIOx->GPIOx_AFRH |= regValue;
		}
	}
}

