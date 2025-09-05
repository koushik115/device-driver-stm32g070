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

void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enable) {
    if (enable) {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_EN();	// Enable Cock for GPIOA
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_EN();	// Enable Cock for GPIOB
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_EN();	// Enable Cock for GPIOC
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_EN();	// Enable Cock for GPIOD
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_EN();	// Enable Cock for GPIOE
        } else if (pGPIOx == GPIOF) {
            GPIOF_PCLK_EN();	// Enable Cock for GPIOF
        }
    } else {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_DS();	// Disable Cock for GPIOA
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_DS();	// Disable Cock for GPIOB
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_DS();	// Disable Cock for GPIOC
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_DS();	// Disable Cock for GPIOD
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_DS();	// Disable Cock for GPIOE
        } else if (pGPIOx == GPIOF) {
            GPIOF_PCLK_DS();	// Disable Cock for GPIOF
        }
    }
}

/*
 * Initialization and De-Initialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t regValue = 0x00;

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

void GPIO_Deinit(GPIO_RegDef_t *pGPIOx) {
    if (pGPIOx == GPIOA) {
    	GPIOA_REG_RESET();  // Reset Register content of GPIOA
    } else if (pGPIOx == GPIOB) {
    	GPIOB_REG_RESET();	// Reset Register content of GPIOB
    } else if (pGPIOx == GPIOC) {
        GPIOC_REG_RESET();	// Reset Register content of GPIOC
    } else if (pGPIOx == GPIOD) {
        GPIOD_REG_RESET();	// Reset Register content of GPIOD
    } else if (pGPIOx == GPIOE) {
        GPIOE_REG_RESET();	// Reset Register content of GPIOE
    } else if (pGPIOx == GPIOF) {
        GPIOF_REG_RESET();	// Reset Register content of GPIOF
    }
}

/*
 * Data Read and Write
 */
uint8_t GPIOx_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	uint8_t pinValue = 0x00;

	// Read the value from GPIOx_IDR register
	pinValue = (uint8_t)((pGPIOx->GPIOx_IDR >> pinNumber) & 0x00000001);

	return pinValue;
}

uint16_t GPIOx_ReadInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t portValue = 0x00;

	// Read the value from GPIOx_IDR register
	portValue = (uint16_t)((pGPIOx->GPIOx_IDR) & 0x0000FFFF);

	return portValue;
}

void GPIOx_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value) {
	pGPIOx->GPIOx_ODR &= ~(0x01 << pinNumber);

	// Write the value into the GPIOx_ODR register
	pGPIOx->GPIOx_ODR |= ((value & 0x01) << pinNumber);
}

void GPIOx_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
	pGPIOx->GPIOx_ODR &= ~(0x0000FFFF);

	// Write the value into the GPIOx_ODR register
	pGPIOx->GPIOx_ODR |= ((value & 0x0000FFFF));
}

void GPIOx_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	pGPIOx->GPIOx_ODR ^= (0x01 << pinNumber);
}
