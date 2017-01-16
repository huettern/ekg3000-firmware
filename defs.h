/*
   EKG3000 - Copyright (C) 2016 FHNW Project 3 Team 2
 */

/**
 * @file       defs.h
 * @brief      Thread and debugging definitions
 * @details 
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       1 December 2016
 * 
 *
 * @addtogroup MAIN
 * @{
 */

#ifndef _DEFS_H
#define _DEFS_H


#define DEFS_THD_IDLE_WA_SIZE 			0x500
#define DEFS_THD_UI_WA_SIZE 			110
#define DEFS_THD_LEDCONTROL_WA_SIZE		512
#define DEFS_THD_ANALOG_WA_SIZE 		512
#define DEFS_THD_SHELL_WA_SIZE 			2048
#define DEFS_THD_WIFIAPP_WA_SIZE 		2048
#define DEFS_THD_ESPRXDBG_WA_SIZE 		256
#define DEFS_THD_WIFICHANNEL_WA_SIZE 	512

#define DEFS_THD_IDLE_NAME 			"main"
#define DEFS_THD_UI_NAME 			"ui"
#define DEFS_THD_LEDCONTROL_NAME	"ledcontrol"
#define DEFS_THD_ANALOG_NAME 		"analog"
#define DEFS_THD_SHELL_NAME 		"shell"
#define DEFS_THD_WIFIAPP_NAME 		"wifiapp"
#define DEFS_THD_ESPRXDBG_NAME 		"esprxdbg"
#define DEFS_THD_WIFICHANNEL_NAME 	"wifichan"


#define DEFS_WIFICHANNEL_DBG	FALSE
#define DEFS_WIFIAPP_DBG		TRUE
#define DEFS_ESP_DBG			TRUE
#define DEFS_ANALOG_DBG			TRUE
#define DEFS_SDC_DBG			FALSE
#define DEFS_UI_DBG				FALSE
#define DEFS_STATE_DBG			FALSE


/*===========================================================================*/
/* Test defines. Always uncomment for production                             */
/*===========================================================================*/

/**
 * Enables the Wifi range test. The devices connects to the MQTT broker and
 * sends "announce" periodically
 */
// #define DEFS_TEST_WIFI_RANGE

/**
 * Sends the testfile validierung.txt from the sdcard to the server
 */
// #define DEFS_TEST_WIFI_FILE


#endif

/** @} */
