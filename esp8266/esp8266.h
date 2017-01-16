/*
   EKG3000 - Copyright (C) 2016 FHNW Project 2 Team 2
 */

/**
 * @file       esp8266.h
 * @brief      Mid level driver for the ESP8266 module
 * @details    Handles the AT-Commands communication over UART with the ESP-01
 *             Source: https://github.com/vpcola/MikroChibiOS
 * WiFi module
 * 
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       1 December 2016
 * 
 *
 * @addtogroup ESP8266
 * @{
 */

#ifndef _ESP8266_H
#define _ESP8266_H

#include "hal.h"

// ESP return values
#define ESP_RET_INVAL           -1
#define ESP_RET_NONE            0x00000001
#define ESP_RET_OK              0x00000002
#define ESP_RET_READY           0x00000004
#define ESP_RET_SENT            0x00000008
#define ESP_RET_ERROR           0x00000010
#define ESP_RET_CONNECT         0x00000020
#define ESP_RET_CLOSED          0x00000040
#define ESP_RET_IPD             0x00000080
#define ESP_RET_SENTFAIL        0x00000100
#define ESP_RET_IP              0x00000200
#define ESP_RET_GATE            0x00000400
#define ESP_RET_NETMA           0x00000800
#define ESP_RET_WIFI_GOT_IP     0x00001000
#define ESP_RET_WIFI_CONNECT    0x00002000
#define ESP_RET_WIFI_DISCONNECT 0x00004000
#define ESP_RET_LINKED          0x00008000
#define ESP_RET_UNLINK          0x00010000


typedef struct {
  uint32_t retval;
  char retstr[100];
} espReturn_t;

enum wifiConStatus {
    ESP_WIFI_CONN_UNKNWN = 1,
    ESP_WIFI_CONN_GOTIP,        // 2
    ESP_WIFI_CONN_CONNECTED,    // 3
    ESP_WIFI_CONN_DISCONNECTED, // 4
};

enum wifiErrors {
    ESP_WIFI_ERR_NONE = 0,
    ESP_WIFI_ERR_AT,
    ESP_WIFI_ERR_RESET,
    ESP_WIFI_ERR_JOIN,
    ESP_WIFI_ERR_MUX,
    ESP_WIFI_ERR_MODE,
    ESP_WIFI_ERR_CONNECT,
    ESP_WIFI_ERR_LINK  
};

enum esp8266ConnectionType
{
    ESP_TCP = 0,
    ESP_UDP
};

typedef struct {
  int id;
  int type;
  char srcaddress[100];
  int port;
  int clisrv;
} espConStatus_t;

#define ESP8266_MAX_CONNECTIONS 5
typedef struct {
  int status[ESP8266_MAX_CONNECTIONS];
} espIPStatus_t;

/*===========================================================================*/
/* Module public functions.                                                  */
/*===========================================================================*/
void espStart(void);
int espInit(void);
bool espHasIP(void);

int espConnectAP(const char *ssid, const char *password);
void espStartWPS(void);

bool esp8266HasData(void);
void esp8266Sleep(bool sl);

// com
int esp8266Server(int channel, int type, uint16_t port);
int esp8266Connect(int channel, const char * ip, uint16_t port, int type);
bool esp8266Disconnect(int channel);

// send
bool esp8266SendLine(int channel, const char * str);
bool esp8266SendHeader(int channel, int datatosend);
bool esp8266Send(const void * data, int msglen, bool waitforok);

// read
uint32_t esp8266ReadRespHeader(int * channel, int * numbytes, int timeout);
int esp8266Read(char * buffer, int bytestoread);
int esp8266Get(int timeout);



#endif 
// _ESP8266_H

/** @} */