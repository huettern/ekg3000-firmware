#include "wifiapp.h"

#include "ch.h"
#include "chprintf.h"

#include "wifichannel.h"
#include "usbcfg.h"
#include "esp8266.h"


/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/
/*
 * defines the thread interval in ms
 */
#define THD_INTERVAL 5000

#define SERVER_IP "192.168.0.178"
#define SERVER_PORT 8266



static THD_WORKING_AREA(waWifi, 256);


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
static char* txmsg;
static int txmsg_size;

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
      chprintf(bssusb, "WIFIAPP:WIFI_DEINIT\r\n");
      if(espInit() == 0) state=WIFI_INITIALIZED;
      break;
    case WIFI_INITIALIZED:
      chprintf(bssusb, "WIFIAPP:WIFI_INITIALIZED\r\n");
      // if(wifiHasIP()) state=WIFI_CONNECTED;
      break;
    case WIFI_CONNECTED:
      chprintf(bssusb, "WIFIAPP:WIFI_CONNECTED\r\n");
      if(blSendDataRdy) state=WIFI_START_TRANSMISSION;
      break;
    case WIFI_START_TRANSMISSION:
      chprintf(bssusb, "WIFIAPP:WIFI_START_TRANSMISSION\r\n");
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

