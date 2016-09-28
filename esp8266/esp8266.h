
#ifndef _ESP8266_H
#define _ESP8266_H

#include "hal.h"

enum wifiErrors {
    WIFI_ERR_NONE = 0,
    WIFI_ERR_AT,
    WIFI_ERR_RESET,
    WIFI_ERR_JOIN,
    WIFI_ERR_MUX,
    WIFI_ERR_MODE,
    WIFI_ERR_CONNECT,
    WIFI_ERR_LINK  
};

enum wifiConRequest {
    WIFI_DISCONNECT = 0,
    WIFI_CONNECT
};

enum wifiConStatus {
    WIFI_CONN_UNKNWN = 1,
    WIFI_CONN_GOTIP,        // 2
    WIFI_CONN_CONNECTED,    // 3
    WIFI_CONN_DISCONNECTED, // 4
};

enum esp8266ConnectionType
{
    TCP = 0,
    UDP
};

// My current esp8266 chip returns
// something like:
// +CWLAP:(3,"SSID",-55,"00:02:6f:d9:9d:18",4
// for each AP found.
#define SSIDMAX_SIZE 50
#define MACADDR_SIZE 20
typedef struct {
  int ecn;
  char ssid[SSIDMAX_SIZE];
  int strength;
  char macaddr[MACADDR_SIZE];
  int unknown;
} APInfo;

#define ESP8266_MAX_CONNECTIONS 5
typedef struct {
  int status[ESP8266_MAX_CONNECTIONS];
} IPStatus;

typedef struct {
  int id;
  int type;
  char srcaddress[100];
  int port;
  int clisrv;
} ConStatus;

#define  RET_INVAL      -1
#define  RET_NONE       0x0001
#define  RET_OK         0x0002
#define  RET_READY      0x0004
#define  RET_LINKED     0x0008
#define  RET_SENT       0x0010
#define  RET_UNLINK     0x0020
#define  RET_ERROR      0x0040
#define  RET_ALREADY_CONNECTED 0x0080
#define  RET_CONNECT    0x0100
#define  RET_CLOSED     0x0200
#define  RET_IPD        0x0400
#define  RET_NOCHANGE   0x0800
#define  RET_SENTFAIL   0x1000

typedef struct {
  int retval;
  char retstr[100];
} EspReturn;


void espInit(void);
void espTerm(char* str);

void espRead(void);

#endif 
// _ESP8266_H