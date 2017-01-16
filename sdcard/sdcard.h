/*
   EKG3000 - Copyright (C) 2016 FHNW Project 3 Team 2
 */

/**
 * @file       sdcard.h
 * @brief      Provides SD Card access and init functinos
 * @details    
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       1 December 2016
 * 
 *
 * @addtogroup SDCARD
 * @{
 */

#ifndef _SDCARD_H
#define _SDCARD_H


#include "hal.h"
#include "ff.h"




/*===========================================================================*/
/* public functions                                                          */
/*===========================================================================*/

void sdcInit(void);
void sdcTree(BaseSequentialStream *chp);

void sdcUnmount (void);
void sdcMount (void);
bool sdcIsMounted (void);
FRESULT sdcMkfs (void);
void sdcBenchmark (BaseSequentialStream *chp, uint8_t it);

#endif //#ifndef _SDCARD_H

/** @} */
