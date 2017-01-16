/*
   EKG3000 - Copyright (C) 2016 FHNW Project 2 Team 2
 */

/**
 * @file       ui.c
 * @brief      User interface functions
 * @details    Provides user interface functions, debounces the buttons
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       1 December 2016
 * 
 *
 * @addtogroup UI
 * @brief User interface routines
 * @{
 */

#include "ui.h"

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "util.h"
#include "defs.h"

#include "usbcfg.h"

#include "ff.h"

#include <stdlib.h>
#include <string.h>

static BaseSequentialStream * dbgstrm = bssusb;
#if DEFS_UI_DBG == TRUE
  #define DBG(X, ...)    chprintf(dbgstrm, X, ##__VA_ARGS__ )
#else
  #define DBG(X, ...)
#endif

/*===========================================================================*/
/* settings                                                                */
/*===========================================================================*/

/*===========================================================================*/
/* prototypes                                                                */
/*===========================================================================*/

/*===========================================================================*/
/* private data                                                              */
/*===========================================================================*/
static PWMConfig pwmcfg = {
  10000,                                    /* 10kHz PWM clock frequency.   */
  10,                                    /* Initial PWM freq 1kHz.       */
  NULL,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },
  0,
  0
};

static THD_WORKING_AREA(uiThreadWA, DEFS_THD_UI_WA_SIZE);

static uint16_t s0inputBuf = 0x5555;
static uint16_t s1inputBuf = 0x5555;
static uint16_t s2inputBuf = 0x5555;

static bool s0released = false;
static bool s0pushed = false;
static bool s0IsPushed = false;
static bool s1released = false;
static bool s1pushed = false;
static bool s1IsPushed = false;
static bool s2released = false;
static bool s2pushed = false;
static bool s2IsPushed = false;

/*===========================================================================*/
/* Module static functions.                                                  */
/*===========================================================================*/
/**
 * @brief      Thread routine debounces buttons
 */
static THD_FUNCTION(uiThread, arg)
{
  (void)arg;
  chRegSetThreadName(DEFS_THD_UI_NAME);

  while(true)
  {
    // debounce
    s0inputBuf = (s0inputBuf<<1) | palReadPad(UI_S0_PORT, UI_S0_PIN);
    s1inputBuf = (s1inputBuf<<1) | palReadPad(UI_S1_PORT, UI_S1_PIN);
    s2inputBuf = (s2inputBuf<<1) | palReadPad(UI_S2_PORT, UI_S2_PIN);

    // check for pressed / released
    if(s0inputBuf == 0x0000)
    {
      if(s0IsPushed != true) s0pushed = true;
      s0IsPushed = true;
    }
    if(s0inputBuf == 0xffff)
    {
      if(s0IsPushed != false) s0released = true;
      s0IsPushed = false;
    }
    if(s1inputBuf == 0x0000)
    {
      if(s1IsPushed != true) s1pushed = true;
      s1IsPushed = true;
    }
    if(s1inputBuf == 0xffff)
    {
      if(s1IsPushed != false) s1released = true;
      s1IsPushed = false;
    }
    if(s2inputBuf == 0x0000)
    {
      if(s2IsPushed != true) s2pushed = true;
      s2IsPushed = true;
    }
    if(s2inputBuf == 0xffff)
    {
      if(s2IsPushed != false) s2released = true;
      s2IsPushed = false;
    }

    chThdSleepMilliseconds(1);
  }
}

/*===========================================================================*/
/* Module public functions.                                                  */
/*===========================================================================*/
/**
 * @brief      Inits the user interface module
 */
void uiInit(void)
{
  // Clear outputs
  palClearPad(UI_LED0_PORT, UI_LED0_PIN);
  palClearPad(UI_LED1_PORT, UI_LED1_PIN);
  palClearPad(UI_LED2_PORT, UI_LED2_PIN);
  palClearPad(UI_LED3_PORT, UI_LED3_PIN);

  // LED as output
  // palSetPadMode(UI_LED0_PORT, UI_LED0_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  // palSetPadMode(UI_LED1_PORT, UI_LED1_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  // palSetPadMode(UI_LED2_PORT, UI_LED2_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  // palSetPadMode(UI_LED3_PORT, UI_LED3_PIN, PAL_MODE_OUTPUT_PUSHPULL);

  // timer config for led PWM
  pwmStart(&PWMD4, &pwmcfg);
  palSetPadMode(UI_LED0_PORT, UI_LED0_PIN, PAL_MODE_ALTERNATE(2));
  palSetPadMode(UI_LED1_PORT, UI_LED1_PIN, PAL_MODE_ALTERNATE(2));
  palSetPadMode(UI_LED2_PORT, UI_LED2_PIN, PAL_MODE_ALTERNATE(2));
  palSetPadMode(UI_LED3_PORT, UI_LED3_PIN, PAL_MODE_ALTERNATE(2));

  pwmEnableChannel(&PWMD4, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, 0));
  pwmEnableChannel(&PWMD4, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, 0));
  pwmEnableChannel(&PWMD4, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, 0));
  pwmEnableChannel(&PWMD4, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, 0));
  
  // Buttons input
  palSetPadMode(UI_S0_PORT, UI_S0_PIN, PAL_MODE_INPUT);
  palSetPadMode(UI_S1_PORT, UI_S1_PIN, PAL_MODE_INPUT);
  palSetPadMode(UI_S2_PORT, UI_S2_PIN, PAL_MODE_INPUT);
  

  // start thread
  (void)chThdCreateStatic(uiThreadWA, sizeof(uiThreadWA),
                           NORMALPRIO, uiThread, NULL);

}

/**
 * @brief      Reads the pushed value of S0
 *
 * @return     Returns true if the button was pushed
 */
bool gets0pushed (void) 
{
  bool val;
  val = s0pushed;
  s0pushed = false;
  return val;
}

/**
 * @brief      Reads the pushed value of S1
 *
 * @return     Returns true if the button was pushed
 */
bool gets1pushed (void) 
{
  bool val;
  val = s1pushed;
  s1pushed = false;
  return val;
}


/**
 * @brief      Reads the pushed value of S2
 *
 * @return     Returns true if the button was pushed
 */
bool gets2pushed (void) 
{
  bool val;
  val = s2pushed;
  s2pushed = false;
  return val;
}

/** @} */
