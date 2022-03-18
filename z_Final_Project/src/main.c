#include <stdint.h>
#include <stdbool.h>
#include "app.h"
#include "cpu.h"
#include "cmu.h"
#include "em_chip.h"
#include "os.h"
#include "bsp_os.h"
#include "os_trace.h"
#include "em_emu.h"

//// BUTTON 0 is
//#define BUTTON0_port gpioPortF
//#define BUTTON0_pin  6u
//#define BUTTON0_default false // Default false (0) = not pressed, true (1) = pressed
//// BUTTON 1 is
//#define BUTTON1_port gpioPortF
//#define BUTTON1_pin  7u
//#define BUTTON1_default false // Default false (0) = not pressed, true (1) = pressed

int main(void)
{
  EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_DEFAULT;
  CMU_HFXOInit_TypeDef hfxoInit = CMU_HFXOINIT_DEFAULT;

  /* Chip errata */
  CHIP_Init();

  /* Init DCDC regulator and HFXO with kit specific parameters */
  /* Init DCDC regulator and HFXO with kit specific parameters */
  /* Initialize DCDC. Always start in low-noise mode. */
  EMU_EM23Init_TypeDef em23Init = EMU_EM23INIT_DEFAULT;
  EMU_DCDCInit(&dcdcInit);
  em23Init.vScaleEM23Voltage = emuVScaleEM23_LowPower;
  EMU_EM23Init(&em23Init);
  CMU_HFXOInit(&hfxoInit);

  /* Switch HFCLK to HFRCO and disable HFRCO */
  CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
  CMU_OscillatorEnable(cmuOsc_HFXO, false, false);

  cmu_open();

  BSP_SystemInit();                                           /* Initialize System.                                   */

  RTOS_ERR err;
  CPU_Init();
  OS_TRACE_INIT();
  OSInit(&err);                                               /* Initialize the Kernel.                               */
  /*   Check error code.                                  */
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

//  OSStart(&err);
  // ------------ Added Code From Part 2 --------------- //
  // disable interrupts before configuring GPIO interrupt
  GPIO_IntDisable(GPIO_IF_EXT_DEFAULT);
  GPIO_IntClear(GPIO_IF_EXT_DEFAULT);
  //                port          pin          intNum       rise  fall  enable
  GPIO_ExtIntConfig(5, 6u, 6u, false, true, true);
  GPIO_ExtIntConfig(5, 7u, 7u, false, true, true);

  app_init();

  // lastly, have to set IRQ bit in NVIC
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);

  /* Start the kernel.                                    */
  OSStart(&err);
  while(1) {
//      EMU_EnterEM1();
  }

}
