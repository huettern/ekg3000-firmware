/*
   EKG3000 - Copyright (C) 2016 FHNW Project 3 Team 2
 */

/**
 * @file       main.c
 * @brief      Main routine and LED control
 * @details    
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       1 December 2016
 * 
 *
 * @addtogroup MAIN
 * @brief main routine and UX
 * @{
 */

#include "ch.h"
#include "hal.h"

#include "wifichannel.h"
#include "ui.h"
#include "defs.h"
#include "wifiapp.h"
#include "sdcard.h"
#include "analog.h"
#include "state.h"

#include "chprintf.h"

#include "usbcdc.h"
#include "usbcfg.h"

/*===========================================================================*/
/* settings                                                                */
/*===========================================================================*/


/*===========================================================================*/
/* private data                                                              */
/*===========================================================================*/
/*
 * Working area for the State machine thread
 */
static THD_WORKING_AREA(ledControlWA, DEFS_THD_LEDCONTROL_WA_SIZE);
 
/**
 * @brief      state machine control thread.
 *
 * @param[in]  <unnamed>  name
 * @param[in]  <unnamed>  arguments
 */
static THD_FUNCTION(ledControlThread, arg) {
  (void)arg;
  chRegSetThreadName(DEFS_THD_LEDCONTROL_NAME);
  while (true) {
    stateRun();
    chThdSleepMilliseconds(STATE_MACHINE_INTERVAL_MS);
  }
}

/*===========================================================================*/
/* prototypes                                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module static functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* callbacks                                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module public functions.                                                  */
/*===========================================================================*/
/*
 * Application entry point.
 */
int main(void) {
  /*
  * System initializations.
  */
  halInit();
  chSysInit();

  /**
  * User init
  */
  uiInit();
  usbcdcInit();
  sdcInit();
  wifiStart();
  anInit();
  anLED(true);
  stateInit();

  /*
  * Creates the LED control
  */
  (void)chThdCreateStatic(ledControlWA, sizeof(ledControlWA),
                   NORMALPRIO, ledControlThread, NULL);

  chRegSetThreadName(DEFS_THD_IDLE_NAME);
  while (true) 
  {
    usbcdcHandleShell();
    chThdSleepMilliseconds(100);
    if(gets0pushed()) 
    {
      if(wifiGetState() == WIFI_SLEEP)
      {
        anSampleT(60);
      }
    }
    if(gets1pushed()) 
    {
      if(wifiGetState() == WIFI_SLEEP)
      {
        anSampleT(5);
      }
    }
  }
}

/** @} */
