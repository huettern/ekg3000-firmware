/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"

#include "chprintf.h"

#define usb_lld_connect_bus(usbp)
#define usb_lld_disconnect_bus(usbp)


#include "usbcfg.h"

#define usb_lld_connect_bus(usbp)
#define usb_lld_disconnect_bus(usbp)

/* Virtual serial port over USB.*/
SerialUSBDriver SDU1;

/*
 * Working area for the LED flashing thread.
 */
static THD_WORKING_AREA(myThreadWorkingArea, 128);
 
/*
 * LED flashing thread.
 */
static THD_FUNCTION(myThread, arg) {
 
  while (true) {
    palSetPad(GPIOB, 0);
    chprintf((BaseSequentialStream *)&SDU1, "Hoi Sven\n\r");
    chThdSleepMilliseconds(50);
    palClearPad(GPIOB, 0);
    // chprintf((BaseSequentialStream *)&SDU1, "Hoi Sven\n\r");
    chThdSleepMilliseconds(50);
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();


  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1000);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  /*
   * Activates the serial driver 2 using the default configuration, pins
   * are pre-configured in board.h.
   */
  // sdStart(&SD2, NULL);

  /*
   * Creates the example thread.
   */

  palSetPadMode(GPIOB, 0, PAL_MODE_OUTPUT_PUSHPULL); /* PD4 */

   /* Starting the flashing LEDs thread.*/
  (void)chThdCreateStatic(myThreadWorkingArea, sizeof(myThreadWorkingArea),
                          NORMALPRIO, myThread, NULL);
  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state, when the button is
   * pressed the test procedure is launched.
   */
  while (true) {
    // palSetPad(GPIOB, 0);
    // chprintf((BaseSequentialStream *)&SDU1, "Hoi Sven\n\r");
    // chThdSleepMilliseconds(500);
    // palClearPad(GPIOB, 0);
    // chprintf((BaseSequentialStream *)&SDU1, "Hoi Sven\n\r");
    // chThdSleepMilliseconds(500);
    chThdSleepMilliseconds(1000);
  }
}
