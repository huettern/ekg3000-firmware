/*
   EKG3000 - Copyright (C) 2016 FHNW Project 3 Team 2
 */

/**
 * @file       state.h
 * @brief      State machine and LED control
 * @details    
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       12 Jan 2017
 * 
 *
 * @addtogroup MAIN
 * @{
 */

#ifndef _STATE_H
#define _STATE_H

#include "hal.h"

/*===========================================================================*/
/* settings                                                                  */
/*===========================================================================*/
/**
 * Define the intervall in which the stateRun function is called
 */
#define STATE_MACHINE_INTERVAL_MS 10

/*===========================================================================*/
/* typedefs                                                                  */
/*===========================================================================*/

typedef enum {
    STATE_INITIAL_CONNECT,
    STATE_WPS,
    STATE_DEFAULT_AP,
    STATE_READY,
    STATE_RECORD,
    STATE_CONNECT,
    STATE_TRANSMIT,
    STATE_FAULT
} state_states;

/*===========================================================================*/
/* modul public functions                                                    */
/*===========================================================================*/

void stateInit(void);
void stateRun(void);



#endif //#ifndef _STATE_H

/** @} */