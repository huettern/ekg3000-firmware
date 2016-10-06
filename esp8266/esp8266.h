
#ifndef _ESP8266_H
#define _ESP8266_H

#include "hal.h"


#define  ESP_RET_INVAL      -1
#define  ESP_RET_NONE       0x0001
#define  ESP_RET_OK         0x0002
#define  ESP_RET_READY      0x0004
#define  ESP_RET_LINKED     0x0008
#define  ESP_RET_SENT       0x0010
#define  ESP_RET_UNLINK     0x0020
#define  ESP_RET_ERROR      0x0040
#define  ESP_RET_ALREADY_CONNECTED 0x0080
#define  ESP_RET_CONNECT    0x0100
#define  ESP_RET_CLOSED     0x0200
#define  ESP_RET_IPD        0x0400
#define  ESP_RET_NOCHANGE   0x0800
#define  ESP_RET_SENTFAIL   0x1000

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


void espStart(void);

int espInit(void);

bool espHasIP(void);


int espConnectAP(const char *ssid, const char *password);

void espTerm(char* str);

void espRead(void);

#endif 
// _ESP8266_H