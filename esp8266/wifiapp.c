#include "wifiapp.h"

#include "ch.h"
#include "chprintf.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "wifichannel.h"
#include "usbcfg.h"
#include "esp8266.h"


/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/
/*
 * defines the thread interval in ms
 */
#define THD_INTERVAL 3000

#define SERVER_IP "192.168.0.178"
#define SERVER_PORT 8266

// #define WIFI_SSID "host huetter tom"
// #define WIFI_PW "kaffeetasse"

#define WIFI_SSID "UPCB91F9C8"
#define WIFI_PW "n7eZvztsrdfk"


static THD_WORKING_AREA(waWifi, 256);


static BaseSequentialStream * dbgstrm = bssusb;
#define DBG(X, ...)    chprintf(dbgstrm, X, ##__VA_ARGS__ )
// #define DBG(X, ...)

/*===========================================================================*/
/* Module local function prototypes                                          */
/*===========================================================================*/
static void fWiFiStateMachine(void);

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/
static thread_t *tp;
static wifi_state state = WIFI_DEINIT;
static binary_semaphore_t wifi_bsem;

static bool blSendDataRdy = false;
static bool blConnectAP = false;

static char* txmsg;
static int txmsg_size;

#define WIFI_SSID_PW_SIZE 25
static char wifi_ssid[WIFI_SSID_PW_SIZE];
static char wifi_pw[WIFI_SSID_PW_SIZE];

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/


/*
 * handles wifi statemachine, called periodically
 */
static void fWiFiStateMachine(void)
{
  switch(state)
  {
    case WIFI_DEINIT:
      if(wifiInit() == ESP_RET_OK) state=WIFI_INITIALIZED;
      break;
    case WIFI_INITIALIZED:
      if(blConnectAP) 
      {
        wifiConnectAP(wifi_ssid, wifi_pw);
        blConnectAP = false;
      }
      else if(wifiHasIP()) state=WIFI_CONNECTED;
      break;
    case WIFI_CONNECTED:
      if(blSendDataRdy) state=WIFI_START_TRANSMISSION;
      break;
    case WIFI_START_TRANSMISSION:
      // channelSendTo(channelOpen(TCP),txmsg,txmsg_size,SERVER_IP,SERVER_PORT);
      blSendDataRdy = false;
      state=WIFI_CONNECTED;
      break;
  }
}

/*
 * Main WiFi application thread
 */
static THD_FUNCTION(thWiFiApp, arg) {
 (void)arg;
 while(true)
 {
  fWiFiStateMachine();
  chThdSleepMilliseconds(THD_INTERVAL);
 }
}


/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/
void wifiAddAP(const char * ssid, const char * password)
{
  if( (strlen(ssid) > WIFI_SSID_PW_SIZE) || (strlen(password) > WIFI_SSID_PW_SIZE) )
  {
    DBG("FATAL wifiConnectAP buf overflow");
    return;
  }
  memcpy(wifi_ssid, ssid, strlen(ssid));
  memcpy(wifi_pw, password, strlen(password));
  blConnectAP = true;
}

/**
 * @brief      Connect to the default AP
 */
void wifiConnectDefaultAP(void)
{
  memcpy(wifi_ssid, WIFI_SSID, strlen(WIFI_SSID));
  memcpy(wifi_pw, WIFI_PW, strlen(WIFI_PW));
  blConnectAP = true;
}

/*
 * starts the wifi thread
 */
void wifiStart(void)
{
  chBSemObjectInit(&wifi_bsem, false);
  tp = chThdCreateStatic(waWifi, sizeof(waWifi), 30, thWiFiApp, NULL);
}


/*
 * transmit raw data to the server
 */
void wifiTransmitRaw(char* buf, int len)
{
  chBSemWait(&wifi_bsem);

  txmsg_size = len;
  memcpy(txmsg, buf, txmsg_size);
  blSendDataRdy = true;

  chBSemSignal(&wifi_bsem);
}

