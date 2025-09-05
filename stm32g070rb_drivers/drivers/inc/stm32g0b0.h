/*
 * stm32g0b0.h
 *
 *  Created on: September 1, 2025
 *      Author: Koushik Bhat
 */

#ifndef INC_STM32G0B0_H_
#define INC_STM32G0B0_H_


#include <stdint.h>


/*
 * Generic Macros
 */
#define SET							1
#define RESET						0
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET
#define ENABLE						SET
#define DISABLE						RESET


/*
 * Memory Base Addresses
 */
#define FLASH_BASE_ADDR					0x08000000UL
#define SRAM_BASE_ADDR					0x20000000UL
#define SYSTEM_MEMORY_BASE_ADDE			0x1FFF0000UL

/*
 * =============================================================================
 *  AHBx and APBx Peripheral Base Addresses
 * =============================================================================
 */
#define APB1_PERIPH_BASE        0x40000000UL
#define APB2_PERIPH_BASE        0x40010000UL
#define AHB_PERIPH_BASE         0x40020000UL
#define IOPORT_PERIPH_BASE      0x50000000UL

/*
 * =============================================================================
 *  APB1 Peripheral Base Addresses
 * =============================================================================
 */
#define TIM3_BASE               (APB1_PERIPH_BASE + 0x0400UL)
#define TIM4_BASE               (APB1_PERIPH_BASE + 0x0800UL)
#define TIM6_BASE               (APB1_PERIPH_BASE + 0x1000UL)
#define TIM7_BASE               (APB1_PERIPH_BASE + 0x1400UL)
#define TIM14_BASE              (APB1_PERIPH_BASE + 0x2000UL)
#define RTC_BASE                (APB1_PERIPH_BASE + 0x2800UL)
#define WWDG_BASE               (APB1_PERIPH_BASE + 0x2C00UL)
#define IWDG_BASE               (APB1_PERIPH_BASE + 0x3000UL)
#define SPI2_BASE               (APB1_PERIPH_BASE + 0x3800UL)
#define SPI3_BASE               (APB1_PERIPH_BASE + 0x3C00UL)
#define USART2_BASE             (APB1_PERIPH_BASE + 0x4400UL)
#define USART3_BASE             (APB1_PERIPH_BASE + 0x4800UL)
#define USART4_BASE             (APB1_PERIPH_BASE + 0x4C00UL)
#define USART5_BASE             (APB1_PERIPH_BASE + 0x5000UL)
#define I2C1_BASE               (APB1_PERIPH_BASE + 0x5400UL)
#define I2C2_BASE               (APB1_PERIPH_BASE + 0x5800UL)
#define USB_BASE                (APB1_PERIPH_BASE + 0x5C00UL)
#define PWR_BASE                (APB1_PERIPH_BASE + 0x7000UL)
#define I2C3_BASE               (APB1_PERIPH_BASE + 0x8800UL)
#define TAMP_BASE               (APB1_PERIPH_BASE + 0xB000UL)

/*
 * =============================================================================
 *  APB2 Peripheral Base Addresses
 * =============================================================================
 */
#define SYSCFG_BASE             (APB2_PERIPH_BASE + 0x0000UL)
#define ITLINE_BASE             (APB2_PERIPH_BASE + 0x0200UL)
#define ADC_BASE                (APB2_PERIPH_BASE + 0x2400UL)
#define TIM1_BASE               (APB2_PERIPH_BASE + 0x2C00UL)
#define SPI1_BASE               (APB2_PERIPH_BASE + 0x3000UL)
#define USART1_BASE             (APB2_PERIPH_BASE + 0x3800UL)
#define USART6_BASE             (APB2_PERIPH_BASE + 0x3C00UL)
#define TIM15_BASE              (APB2_PERIPH_BASE + 0x4000UL)
#define TIM16_BASE              (APB2_PERIPH_BASE + 0x4400UL)
#define TIM17_BASE              (APB2_PERIPH_BASE + 0x4800UL)
#define DBG_BASE                (APB2_PERIPH_BASE + 0x5800UL)

/*
 * =============================================================================
 *  AHB Peripheral Base Addresses
 * =============================================================================
 */
#define DMA1_BASE               (AHB_PERIPH_BASE + 0x0000UL)
#define DMA2_BASE               (AHB_PERIPH_BASE + 0x0400UL)
#define DMAMUX_BASE             (AHB_PERIPH_BASE + 0x0800UL)
#define RCC_BASE                (AHB_PERIPH_BASE + 0x1000UL)
#define EXTI_BASE               (AHB_PERIPH_BASE + 0x1800UL)
#define FLASH_BASE              (AHB_PERIPH_BASE + 0x2000UL)
#define CRC_BASE                (AHB_PERIPH_BASE + 0x3000UL)

/*
 * =============================================================================
 *  IOPORT Peripheral Base Addresses (GPIO)
 * =============================================================================
 */
#define GPIOA_BASE              (IOPORT_PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE              (IOPORT_PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE              (IOPORT_PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE              (IOPORT_PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE              (IOPORT_PERIPH_BASE + 0x1000UL)
#define GPIOF_BASE              (IOPORT_PERIPH_BASE + 0x1400UL)

/*
 * =============================================================================
 *  Peripheral Register Structures
 * =============================================================================
 */

// RCC (Reset Control and Clock)
typedef struct {
	volatile uint32_t RCC_CR;
	volatile uint32_t RCC_ICSCR;
	volatile uint32_t RCC_CFGR;
	volatile uint32_t RCC_PLLCFGR;
	volatile uint32_t RESERVED1;
	volatile uint32_t RESERVED2;
	volatile uint32_t RCC_CIER;
	volatile uint32_t RCC_CIFR;
	volatile uint32_t RCC_CICR;
	volatile uint32_t RCC_IOPRSTR;
	volatile uint32_t RCC_AHBRSTR;
	volatile uint32_t RCC_APBRSTR1;
	volatile uint32_t RCC_APBRSTR2;
	volatile uint32_t RCC_IOPENR;
	volatile uint32_t RCC_AHBENR;
	volatile uint32_t RCC_APBENR1;
	volatile uint32_t RCC_APBENR2;
	volatile uint32_t RCC_IOPSMENR;
	volatile uint32_t RCC_AHBSMENR;
	volatile uint32_t RCC_APBSMENR1;
	volatile uint32_t RCC_APBSMENR2;
	volatile uint32_t RCC_CCIPR;
	volatile uint32_t RCC_CCIPR2;
	volatile uint32_t RCC_BDCR;
	volatile uint32_t RCC_CSR;
}RCC_RegDef_t;

// GPIOx Registers
typedef struct {
	volatile uint32_t GPIOx_MODER;
	volatile uint32_t GPIOx_OTYPER;
	volatile uint32_t GPIOx_OSPEEDR;
	volatile uint32_t GPIOx_PUPDR;
	volatile uint32_t GPIOx_IDR;
	volatile uint32_t GPIOx_ODR;
	volatile uint32_t GPIOx_BSRR;
	volatile uint32_t GPIOx_LCKR;
	volatile uint32_t GPIOx_AFRL;
	volatile uint32_t GPIOx_AFRH;
	volatile uint32_t GPIOx_BRR;
}GPIO_RegDef_t;

/*
 * =============================================================================
 *  Peripheral Definitions
 * =============================================================================
 */

#define RCC 			((RCC_RegDef_t *)RCC_BASE)

#define GPIOA			((GPIO_RegDef_t *)GPIOA_BASE)
#define GPIOB			((GPIO_RegDef_t *)GPIOB_BASE)
#define GPIOC			((GPIO_RegDef_t *)GPIOC_BASE)
#define GPIOD			((GPIO_RegDef_t *)GPIOD_BASE)
#define GPIOE			((GPIO_RegDef_t *)GPIOE_BASE)
#define GPIOF			((GPIO_RegDef_t *)GPIOF_BASE)


#define GPIOA_PCLK_EN()	(RCC->RCC_IOPENR |= (1 << 0))
#define GPIOB_PCLK_EN()	(RCC->RCC_IOPENR |= (1 << 1))
#define GPIOC_PCLK_EN()	(RCC->RCC_IOPENR |= (1 << 2))
#define GPIOD_PCLK_EN()	(RCC->RCC_IOPENR |= (1 << 3))
#define GPIOE_PCLK_EN()	(RCC->RCC_IOPENR |= (1 << 4))
#define GPIOF_PCLK_EN()	(RCC->RCC_IOPENR |= (1 << 5))

#define GPIOA_PCLK_DS()	(RCC->RCC_IOPENR &= ~(1 << 0))
#define GPIOB_PCLK_DS()	(RCC->RCC_IOPENR &= ~(1 << 1))
#define GPIOC_PCLK_DS()	(RCC->RCC_IOPENR &= ~(1 << 2))
#define GPIOD_PCLK_DS()	(RCC->RCC_IOPENR &= ~(1 << 3))
#define GPIOE_PCLK_DS()	(RCC->RCC_IOPENR &= ~(1 << 4))
#define GPIOF_PCLK_DS()	(RCC->RCC_IOPENR &= ~(1 << 5))

#define GPIOA_REG_RESET() do { RCC->RCC_IOPRSTR |=  (1 << 0); RCC->RCC_IOPRSTR &= ~(1 << 0); } while(0)
#define GPIOB_REG_RESET() do { RCC->RCC_IOPRSTR |=  (1 << 1); RCC->RCC_IOPRSTR &= ~(1 << 1); } while(0)
#define GPIOC_REG_RESET() do { RCC->RCC_IOPRSTR |=  (1 << 2); RCC->RCC_IOPRSTR &= ~(1 << 2); } while(0)
#define GPIOD_REG_RESET() do { RCC->RCC_IOPRSTR |=  (1 << 3); RCC->RCC_IOPRSTR &= ~(1 << 3); } while(0)
#define GPIOE_REG_RESET() do { RCC->RCC_IOPRSTR |=  (1 << 4); RCC->RCC_IOPRSTR &= ~(1 << 4); } while(0)
#define GPIOF_REG_RESET() do { RCC->RCC_IOPRSTR |=  (1 << 5); RCC->RCC_IOPRSTR &= ~(1 << 5); } while(0)


#endif /* INC_STM32G0B0_H_ */
