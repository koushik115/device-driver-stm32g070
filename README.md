# STM32G070RB Driver Development

This repository contains bare-metal driver development for the **STM32G070RB SoC**, written from scratch without using HAL/LL drivers.  
The goal is to build a complete set of peripheral drivers for learning and deeper understanding of ARM Cortex-M0+ microcontrollers.

##  Completed
- **GPIO Driver**
  - Configure pin modes (Input, Output, Alternate, Analog)
  - Configure pull-up/pull-down resistors
  - Read pin/port input values
  - Write pin/port output values
  - Toggle pins
  - De-initialize GPIO peripherals
  - Peripheral clock control

##  Project Structure
├── drivers
│ ├── inc
│ │ └── stm32g070_gpio_driver.h # GPIO driver header
│ ├── src
│ │ └── stm32g070_gpio_driver.c # GPIO driver source
├── Core
│ ├── Src
│ │ └── main.c # Example usage


##  Example Usage
#include "stm32g070_gpio_driver.h"

int main(void) {
    GPIO_Handle_t gpioLed;

    gpioLed.pGPIOx = GPIOA;
    gpioLed.GPIO_PinConfig.GPIO_PinNumber = 5;          // PA5 (on-board LED)
    gpioLed.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_OUT;
    gpioLed.GPIO_PinConfig.GPIO_PinSpeed  = GPIO_SPEED_HIGH;
    gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeripheralClockControl(GPIOA, ENABLE);
    GPIO_Init(&gpioLed);

    while(1) {
        GPIO_ToggleOutputPin(GPIOA, 5);
        for (volatile int i = 0; i < 100000; i++);  // simple delay
    }
}

## Requirements

STM32CubeIDE or ARM GCC Toolchain

STM32G070RB Nucleo board (or compatible board)

## License

This project is for educational purposes and open for learning.
Feel free to fork, modify, and contribute.
