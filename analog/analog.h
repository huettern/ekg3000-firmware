/*
   EKG3000 - Copyright (C) 2016 FHNW Project 3 Team 2
 */

/**
 * @file       analog.h
 * @brief      Analog module for ECG signal conversion
 * @details    
 * 
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       1 December 2016
 * 
 *
 * @addtogroup ANALOG
 * @{
 */

#ifndef _ANALOG_H
#define _ANALOG_H

#include "hal.h"

/*===========================================================================*/
/* public function prototypes                                                */
/*===========================================================================*/
void anInit(void);
void anLED(bool on);
void anSampleN(uint32_t n);
void anSampleT(uint32_t t);
bool anIsSampling(void);

#endif //#ifndef _ANALOG_H

/** @} */
