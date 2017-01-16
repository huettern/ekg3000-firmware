/*
   EKG3000 - Copyright (C) 2016 FHNW Project 2 Team 2
 */

/**
 * @file       wifiapp.h
 * @brief      WiFi application thread for MQTT communication and data transfer
 * @details    
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       1 December 2016
 * 
 *
 * @addtogroup WIFIAPP
 * @{
 */

#ifndef __WIFIAPP_H
#define __WIFIAPP_H

#include "hal.h"



/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef enum {
    WIFI_APP_DEINIT = 0,
    WIFI_DEINIT,
    WIFI_INITIALIZED,
    WIFI_CONNECTED,
    WIFI_START_TRANSMISSION,
    WIFI_WPS,
    WIFI_BULK_TRANSFER,
    WIFI_SLEEP,
    WIFI_WAKE_UP,
    WIFI_TEST_RANGE,
    WIFI_TEST_FILE
} wifi_state;


/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void wifiStart(void);


void wifiConnectDefaultAP(void);
void wifiAddAP(const char * ssid, const char * password);
void wifiPublish(const char* msg);
void wifiStartWPS(void);
void wifiStartBulkTransfer(char* fname, uint16_t samplerate);
void wifiSleep(void);
wifi_state wifiGetState (void);

#endif


/** @} */