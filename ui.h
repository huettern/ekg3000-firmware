/*
   EKG3000 - Copyright (C) 2016 FHNW Project 3 Team 2
 */

/**
 * @file       ui.h
 * @brief      User interface functions
 * @details    
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       1 December 2016
 * 
 *
 * @addtogroup UI
 * @{
 */

#ifndef _UI_H
#define _UI_H

#include "hal.h"

/*===========================================================================*/
/* settings                                                                  */
/*===========================================================================*/
#define UI_LED0_PORT GPIOB
#define UI_LED0_PIN 6
#define UI_LED1_PORT GPIOB
#define UI_LED1_PIN 7
#define UI_LED2_PORT GPIOB
#define UI_LED2_PIN 8
#define UI_LED3_PORT GPIOB
#define UI_LED3_PIN 9

#define UI_S0_PORT GPIOB
#define UI_S0_PIN 2
#define UI_S1_PORT GPIOE
#define UI_S1_PIN 8
#define UI_S2_PORT GPIOE
#define UI_S2_PIN 9

/*===========================================================================*/
/* macros                                                                    */
/*===========================================================================*/
/** Set led brightness from I-Locked state */
#define UI_SET_LED0I(x) pwmEnableChannelI(&PWMD4, 0, \
                        PWM_PERCENTAGE_TO_WIDTH(&PWMD4, x*100))
/** Set led brightness from I-Locked state */
#define UI_SET_LED1I(x) pwmEnableChannelI(&PWMD4, 1, \
                        PWM_PERCENTAGE_TO_WIDTH(&PWMD4, x*100))
/** Set led brightness from I-Locked state */
#define UI_SET_LED2I(x) pwmEnableChannelI(&PWMD4, 2, \
                        PWM_PERCENTAGE_TO_WIDTH(&PWMD4, x*100))
/** Set led brightness from I-Locked state */
#define UI_SET_LED3I(x) pwmEnableChannelI(&PWMD4, 3, \
                        PWM_PERCENTAGE_TO_WIDTH(&PWMD4, x*100))

/** Set led brightness from Normal state */
#define UI_SET_LED0(x) pwmEnableChannel(&PWMD4, 0, \
                        PWM_PERCENTAGE_TO_WIDTH(&PWMD4, x*100))
/** Set led brightness from Normal state */
#define UI_SET_LED1(x) pwmEnableChannel(&PWMD4, 1, \
                        PWM_PERCENTAGE_TO_WIDTH(&PWMD4, x*100))
/** Set led brightness from Normal state */
#define UI_SET_LED2(x) pwmEnableChannel(&PWMD4, 2, \
                        PWM_PERCENTAGE_TO_WIDTH(&PWMD4, x*100))
/** Set led brightness from Normal state */
#define UI_SET_LED3(x) pwmEnableChannel(&PWMD4, 3, \
                        PWM_PERCENTAGE_TO_WIDTH(&PWMD4, x*100))
//@}

/*===========================================================================*/
/* modul public functions                                                    */
/*===========================================================================*/

void uiInit(void);

bool gets0pushed (void);
bool gets1pushed (void);
bool gets2pushed (void);
bool gets2longPushed (void);


#endif //#ifndef _UI_H

/** @} */