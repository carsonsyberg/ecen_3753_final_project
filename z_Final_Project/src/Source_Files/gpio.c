//***********************************************************************************
// Include files
//***********************************************************************************
#include "gpio.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************


//***********************************************************************************
// functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 * configures the gpio pins
 *
 * @details
 * sets up drivestrength and pin modes for all in use gpios
 * these include the leds and the buttons
 *
 * @note
 * if any other gpio even interrupt is triggered we want to assert false because none of them should even be enabled
 *
 ******************************************************************************/
void gpio_open(void){

  // Set LED ports to be standard output drive with default off (cleared)
  GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthStrongAlternateStrong);
//  GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
  GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, LED0_default);

  GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthStrongAlternateStrong);
//  GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
  GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, LED1_default);

  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInput, false);
  GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInput, false);
}
