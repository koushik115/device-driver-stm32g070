/*
 * stm32g070_gpio_driver.h
 *
 *  Created on: Sep 4, 2025
 *      Author: Koushik
 */

#ifndef STM32G070_GPIO_DRIVER_H_
#define STM32G070_GPIO_DRIVER_H_

#include "stm32g0b0.h"

typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_OutputType;
	uint8_t GPIO_OutputSpeed;
	uint8_t GPIO_PullUpPullDownConfig;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct {
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*
 * =============================================================================
 *  GPIO Register Definitions
 * =============================================================================
 */
/*
 *  GPIO Pin Number
 */
#define GPIO_PIN_NO_0				0
#define GPIO_PIN_NO_1				1
#define GPIO_PIN_NO_2				2
#define GPIO_PIN_NO_3				3
#define GPIO_PIN_NO_4				4
#define GPIO_PIN_NO_5				5
#define GPIO_PIN_NO_6				6
#define GPIO_PIN_NO_7				7
#define GPIO_PIN_NO_8				8
#define GPIO_PIN_NO_9				9
#define GPIO_PIN_NO_10				10
#define GPIO_PIN_NO_11				11
#define GPIO_PIN_NO_12				12
#define GPIO_PIN_NO_13				13
#define GPIO_PIN_NO_14				14
#define GPIO_PIN_NO_15				15

/*
 * GPIO Mode
 */
#define GPIO_MODE_INPUT				0
#define GPIO_MODE_OUTPUT			1
#define GPIO_MODE_ALT_MODE			2
#define GPIO_MODE_ANALOG			3
#define GPIO_MODE_INT_FALL_EDGE		4
#define GPIO_MODE_INT_RISE_EDGE		5
#define GPIO_MODE_INT_BOTH_EDGE		6

/*
 * GPIO Output Types
 */
#define GPIO_OUTPUT_PUSH_PULL		0
#define GPIO_OUTPUT_OPEN_DRAIN		1

/*
 * GPIO Output Speed
 */

#define GPIO_VERY_LOW_SPEED			0
#define GPIO_LOW_SPEEED				1
#define GPIO_HIGH_SPEED				2
#define GPIO_VERY_HIGH_SPEED		3

/*
 *  GPIO Pull-Up and Pull-Down Configuration
 */
#define GPIO_NO_PULLUP_PULLDOWN		0
#define	GPIO_PULLUP					1
#define GPIO_PULLDOWN				2

/*************************** APIs Supported  by this driver *********************************/

/*
 * Peripheral Clock Control
 */
void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enable);

/*
 * Initialization and De-Initialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Write
 */
uint8_t GPIOx_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIOx_ReadInputPort(GPIO_RegDef_t *pGPIOx);
void GPIOx_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIOx_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIOx_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/*
 * IRQ Configuration and Handling
 */
void GPIOx_IRQConfig(uint8_t IRQn, uint8_t priority, uint8_t enable);
void GPIOx_IRQHandling(uint8_t pinNumber);

#endif /* STM32G070_GPIO_DRIVER_H_ */
