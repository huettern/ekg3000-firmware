/*
   EKG3000 - Copyright (C) 2016 FHNW Project 2 Team 2
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

#include "chprintf.h"

#include "usbcdc.h"
#include "usbcfg.h"

/*===========================================================================*/
/* settings                                                                */
/*===========================================================================*/

/**
 * LED brightness in %
 */
#define LED_BRIGHT 100
#define LED_DARK 0

/*===========================================================================*/
/* private data                                                              */
/*===========================================================================*/
/*
 * Working area for the LED flashing thread.
 */
static THD_WORKING_AREA(ledControlWA, DEFS_THD_LEDCONTROL_WA_SIZE);
 
/*
 * LED control thread.
 */
static THD_FUNCTION(ledControlThread, arg) {
  (void)arg;
  wifi_state wifiState;

  UI_SET_LED0(LED_BRIGHT); // system ready

  chRegSetThreadName(DEFS_THD_LEDCONTROL_NAME);
  while (true) {
    // UI_SET_LED1(); // controlled by analog.c module
    
    
    // Wifi LED
    wifiState = wifiGetState();
    if((wifiState == WIFI_INITIALIZED) || 
       (wifiState == WIFI_CONNECTED) ||
       (wifiState == WIFI_DEINIT) )
      UI_SET_LED2(LED_BRIGHT);
    else if(wifiGetState() == WIFI_BULK_TRANSFER)
      UI_SET_LED2(LED_BRIGHT);
    else
      UI_SET_LED2(LED_DARK);

    if(anIsSampling()) UI_SET_LED3(LED_BRIGHT);
    else UI_SET_LED3(LED_DARK);

    chThdSleepMilliseconds(100);
    UI_SET_LED0(LED_BRIGHT);
    // UI_SET_LED1(); // controlled by analog.c module
    
    // Wifi LED
    wifiState = wifiGetState();
    if((wifiState == WIFI_INITIALIZED) || 
       (wifiState == WIFI_CONNECTED) ||
       (wifiState == WIFI_DEINIT) )
      UI_SET_LED2(LED_DARK);
    else if(wifiGetState() == WIFI_BULK_TRANSFER)
      UI_SET_LED2(LED_BRIGHT);
    else
      UI_SET_LED2(LED_DARK);

    if(anIsSampling()) UI_SET_LED3(LED_BRIGHT);
    else UI_SET_LED3(LED_DARK);
    
    chThdSleepMilliseconds(100);
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
    if(gets2pushed())
    {
      wifiStartWPS();
    }
  }
}

/** @} */
