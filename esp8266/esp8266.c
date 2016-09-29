#include "esp8266.h"
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "util.h"

#include "usbcfg.h"


#include <stdlib.h>
#include <string.h>

static BaseSequentialStream * usart = &SD2;
static BaseSequentialStream * dbgstrm = stdio;

#define DBG(X, ...)    chprintf(stdio, X, ##__VA_ARGS__ )

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

// Reads the 8266 until the matched string
// is read
bool esp8266ReadUntil(const char * resp, int timeout)
{
    int c;
    int index = 0;
    int targetLength = strlen(resp);

    while((c = sdGetTimeout((SerialDriver *) usart, timeout)) >= 0)
    {
        DBG("%c", c);
        if (c != resp[index])
              index = 0;

        if (c == resp[index]){
           if(++index >= targetLength){
                return true;
           }
        }
    };

    return false;
}

static int esp8266ReadSwitch(int retvals, char * buffer, int * bufsiz, int timeout)
{
  int index[NUM_RESPONSES];
  int lens[NUM_RESPONSES];
  int numstored, i, x, c, numlen;

  if (!buffer || !bufsiz) return -1;

  numlen = NUM_RESPONSES;
  for (i = 0; i < numlen; i++)
  {
    index[i] = 0;
    lens[i] = strlen(returnValues[i].retstr);
  }

  numstored = 0;
  while((c = sdGetTimeout((SerialDriver *) usart, timeout)) > 0)
  {
     if (numstored < *bufsiz)
     {
       buffer[numstored] = c;
       numstored++;
       DBG("%c", c);

       // Evaluate if this belongs to the
       // list of return values
       for(x = 0; x < numlen; x++)
       {
         if (c != returnValues[x].retstr[index[x]])  index[x] = 0;
         if (c == returnValues[x].retstr[index[x]])
           if(++(index[x]) >= lens[x])
           {
             if (retvals & returnValues[x].retval)
             {
               buffer[numstored] = 0;
               *bufsiz = numstored;
               DBG("\r\n**Got [%s], returning %d\r\n",
                   buffer, returnValues[x].retval);
               return returnValues[x].retval;
             }
           }
       }

       // Reset the buffer on a newline or linefeed
       if ((numstored >= 2) &&
           (buffer[numstored-2] == '\r') &&
           (buffer[numstored-1] == '\n')
         )
           numstored = 0;

     }else
       break;
  }

  buffer[numstored] = 0;
  *bufsiz = numstored;
  DBG("\r\n**Got [%s], returning with -1\r\n", buffer);
  return -1;
}

static int esp8266ReadBuffUntil(char * buffer, int len, const char * resp, int timeout)
{
    int c;
    int index = 0;
    int numread = 0;
    int targetLength = strlen(resp);

    memset(buffer, 0, len);
    do
    {
        c = sdGetTimeout((SerialDriver *) usart, timeout);
        if (c >= 0) {
            DBG("%c", c);
            if (numread < len)
            {
              buffer[numread] = c;
              numread ++;
            }

            if (c != resp[index])
                index = 0;

            if (c == resp[index]){
                if(++index >= targetLength){
                    // buffer[numread] = 0;
                    return numread;
                }
            }
        }
    }while((c >= 0) && (numread < len));

    return numread;
}

int esp8266ReadLinesUntil(char * resp, responselinehandler handler)
{
  int numlines = 0, numread = 0;
  char linebuff[200];

  numread = esp8266ReadBuffUntil(linebuff, 200, "\r\n", READ_TIMEOUT);
  while(numread > 0)
  {
    // If we found our line
    if (strstr(linebuff, resp) != NULL)
      return numlines;

    rtrim(linebuff, '\r');
    if (handler) handler(linebuff, strlen(linebuff));
    numlines++;
    numread = esp8266ReadBuffUntil(linebuff, 200, "\r\n", READ_TIMEOUT);
  }

  return numlines;
}

bool esp8266Cmd(const char * cmd, const char * rsp, int cmddelay)
{
    if (usart)
    {
        // Send the command to the wifi chip 
        chprintf(usart, "%s\r\n", cmd);
        DBG(">>%s\r\n", cmd);
        if (cmddelay > 0)
            chThdSleepMilliseconds(cmddelay);

        return esp8266ReadUntil(rsp, READ_TIMEOUT);
    }else
        return false;

}

static int esp8266CmdX(const char * cmd, int returns, int cmddelay, int timeout)
{
  int bufsiz = RXBUFF_SIZE;
  if (usart)
  {
      // Send the command to the wifi chip
      chprintf(usart, "%s\r\n", cmd);
      DBG(">>%s\r\n", cmd);
      if (cmddelay > 0)
          chThdSleepMilliseconds(cmddelay);

      return esp8266ReadSwitch(returns, rxbuff, &bufsiz, timeout);
  }
  return -1;
}

int esp8266CmdCallback(const char *cmd, const char * rsp, responselinehandler handler)
{
  chprintf(usart, "%s\r\n", cmd);
  DBG(">>%s\r\n", cmd);
  return esp8266ReadLinesUntil((char *) rsp, handler);
}

static void ongetfirmwareversion(const char * buffer, int len)
{
  char tmpbuf[50];
  strncpy(tmpbuf, buffer, len);
  // Compare only the first 2 numbers, usually "00"
  if(strncmp((const char *) tmpbuf, "00", 2) == 0)
    strcpy(firmwareVersionStr, rtrim(tmpbuf, '\r'));
}

static void onAddAP(APInfo * info)
{
  if (stdio)
  chprintf(stdio, "SSID[%s] signal[%d] ecn[%d] MAC[%s]\r\n",
      info->ssid,
      info->strength,
      info->ecn,
      info->macaddr);
}

static const char * esp8266GetFirmwareVersion(void)
{
  memset(firmwareVersionStr, 0, FW_VERSION_STR_SIZE);
  esp8266CmdCallback("AT+GMR\r\n", "OK\r\n", ongetfirmwareversion);
  return firmwareVersionStr;
}

bool esp8266SetMode(int mode)
{
  chsnprintf(txbuff, 200, "AT+CWMODE=%d\r\n", mode);
  return (esp8266CmdX(txbuff, RET_OK | RET_NOCHANGE, 100, TIME_IMMEDIATE) == RET_OK );
}

int esp8266ListAP(onNewAP apCallback)
{
  int numlines = 0, numread = 0;
  char line[200], *p;
  APInfo apinfo;

  chprintf(stdio, "ESP8266 Listing APs...\r\n");

  chprintf(usart, "AT+CWLAP\r\n");
  DBG(">>AT+CWLAP\r\n");
  while((numread = esp8266ReadBuffUntil(rxbuff, RXBUFF_SIZE, "\r\n", TIME_INFINITE)) > 0)
  {
    // If we found our line
    if (strstr(rxbuff, "OK\r\n") != NULL)
      return numlines;

    rtrim(rxbuff, '\r');
    rtrim(rxbuff, ')' );
    if (strstr(rxbuff, "+CWLAP:(") != NULL)
    {
        strcpy(line, (char *) rxbuff + 8);
        //DBG("AP [%s]\r\n", line);
        p = strtok(line, ",");
        if (p) apinfo.ecn = atoi(p); // ecn
        p = strtok(NULL, ",");
        if (p) strcpy(apinfo.ssid, p);
        p = strtok(NULL, ",");
        if (p) apinfo.strength = atoi(p);
        p = strtok(NULL, ",");
        if (p) strcpy(apinfo.macaddr, p);
        // Don't know what the last param
        // is, it doesn't seem to be documented
        // anywhere

        // Call the callback
        if(apCallback) apCallback(&apinfo);
    }
    numlines++;
  }

  return numlines;
}

int espInit(void) {
  uint32_t ctr;

  chprintf(stdio, "Starting esp init...\r\n");

  /*
   * Activates the serial driver 2 using the driver default configuration.
   * PA2(TX) and PA3(RX) are routed to USART2.
   */
  sdStart(&SD2, &sd2conf);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));
  
  if (!esp8266Cmd("AT+RST\r\n", "AT+RST", 1000))
    {
        chprintf(stdio, "Failed to reset ESP8266!\r\n");
        return WIFI_ERR_RESET;
    }else
        chprintf(stdio, "\r\nESP8266 Initialized\r\n");

  espStatus = WIFI_INITIALIZED;

  chprintf(stdio, "ESP8266 Firmware Version [%s]\r\n", esp8266GetFirmwareVersion());

  if(esp8266SetMode(WIFI_MODE_STA))
      chprintf(stdio, "ESP8266 Mode set to %d\r\n", WIFI_MODE_STA);
  chprintf(stdio, "done\r\n");

  esp8266ListAP(onAddAP);

    return WIFI_ERR_NONE;
  // chprintf(stdio, ">AT+CWMODE=1\r\n",rxbuff);
  // chprintf(&SD2, "AT+CWMODE=1\r\n");

  // chThdSleepMilliseconds(100);
  // ctr = sdReadTimeout(&SD2, rxbuff, 64, TIME_IMMEDIATE);
  // rxbuff[ctr] = 0;
  // chprintf(stdio, "<%s\r\n",rxbuff);


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
  chprintf(stdio, "ret<%s\r\n",rxbuff);
}