#include "esp8266.h"
#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "usbcfg.h"


#include <stdlib.h>
#include <string.h>

static BaseSequentialStream * usart = NULL;
static BaseSequentialStream * dbgstrm = NULL;


#define RXBUFF_SIZE 2048
#define TXBUFF_SIZE 200

#define READ_TIMEOUT 1000 // uart read timeout on 1000 ticks
#define WRITE_TIMEOUT 1000 // uart write timeout on 1000 ticks
// General purpose buffer for reading results
static char rxbuff[RXBUFF_SIZE]  __attribute__ ((section(".bufram")));
static char txbuff[TXBUFF_SIZE]  __attribute__ ((section(".bufram")));

SerialConfig sd2conf = {
  115200
};

#define FW_VERSION_STR_SIZE 100
static char firmwareVersionStr[FW_VERSION_STR_SIZE];
#define IP_STR_SIZE 100
static char assignedIP[IP_STR_SIZE] = {0};

typedef enum {
    WIFI_RESET = 0,
    WIFI_INITIALIZED,
    WIFI_AP_CONNECTED
} WIFI_STATUS;

static const EspReturn returnValues[] = {
   { RET_ALREADY_CONNECTED, "LINK IS BUILDED\r\n" },
   { RET_NOCHANGE,          "no change\r\n" },
   { RET_SENT,              "SEND OK\r\n" },
   { RET_SENTFAIL,          "SEND FAIL\r\n" },
   { RET_ERROR,             "ERROR\r\n" },
   { RET_UNLINK,            "Unlink\r\n" },
   { RET_LINKED,            "Linked\r\n" },
   { RET_CLOSED,            ",CLOSED\r\n" },
   { RET_CONNECT,           ",CONNECT\r\n" },
   { RET_READY,             "ready\r\n" },
   { RET_IPD,               "+IPD,"  },
   { RET_OK,                "OK\r\n" },
   { RET_NONE,              "\r\n" },
};

#define NUM_RESPONSES sizeof(returnValues)/sizeof(EspReturn)

static WIFI_STATUS espStatus = WIFI_RESET;

void espInit(void) {
  uint32_t ctr;

  chprintf(stdio, "Starting esp init...\r\n");

  /*
   * Activates the serial driver 2 using the driver default configuration.
   * PA2(TX) and PA3(RX) are routed to USART2.
   */
  sdStart(&SD2, &sd2conf);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  chprintf(stdio, ">AT+CWMODE=1\r\n",rxbuff);
  chprintf(&SD2, "AT+CWMODE=1\r\n");

  chThdSleepMilliseconds(100);
  ctr = sdReadTimeout(&SD2, rxbuff, 64, TIME_IMMEDIATE);
  rxbuff[ctr] = 0;
  chprintf(stdio, "<%s\r\n",rxbuff);


}


void espTerm(char* str) {
  uint32_t ctr;

  chprintf(stdio, ">%s\r\n",str);
  sdAsynchronousWrite(&SD2, str, strlen(str));
  // chprintf(&SD2, "%s\r\n",str);

  chThdSleepMilliseconds(1000);
  espRead();
  // ctr = sdReadTimeout(&SD2, rxbuff, 64, TIME_IMMEDIATE);
  // rxbuff[ctr%RXBUFF_SIZE] = 0;
  // chprintf(stdio, "<%s\r\n",rxbuff);
}

void espRead() {
  uint32_t ctr;
  ctr = sdReadTimeout(&SD2, rxbuff, RXBUFF_SIZE, TIME_IMMEDIATE);
  // ctr = sdRead(&SD2, rxbuff, RXBUFF_SIZE);
  rxbuff[ctr%RXBUFF_SIZE] = 0;
  chprintf(stdio, "<%s\r\n",rxbuff);
}