/*
   EKG3000 - Copyright (C) 2016 FHNW Project 2 Team 2
 */

/**
 * @file       esp8266.c
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
 * @brief esp8266 AT Command driver
 * @{
 */

#include "esp8266.h"
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "util.h"
#include "wifichannel.h"
#include "defs.h"

#include "usbcfg.h"

#include <stdlib.h>
#include <string.h>

static BaseSequentialStream * dbgstrm = bssusb;
#if DEFS_ESP_DBG == TRUE
  #define DBG(X, ...)    chprintf(dbgstrm, X, ##__VA_ARGS__ )
#else
  #define DBG(X, ...)
#endif

/*===========================================================================*/
/* settings                                                                */
/*===========================================================================*/
// UART device to use
#define uart &UARTD2

#define ESP_PD_PORT GPIOA
#define ESP_PD_PIN 1

// Allows max MTU size of 1500
#define RXBUFF_SIZE 1600
#define TXBUFF_SIZE 1500
#define LINEBUFF_SIZE 1600

#define READ_UNTIL_TIMEOUT_US 50 // wait time before each character read
#define WRITE_TIMEOUT 1000 // uart write timeout on 1000 ticks

/*===========================================================================*/
/* prototypes                                                                */
/*===========================================================================*/
static void txend1(UARTDriver *uartp);
static void txend2(UARTDriver *uartp);
static void rxerr(UARTDriver *uartp, uartflags_t e);
static void rxchar(UARTDriver *uartp, uint16_t c);
static void rxend(UARTDriver *uartp);

static void parseIP(void);
static void parseGateway(void);
static void parseNetmask(void);


/*===========================================================================*/
/* private data                                                              */
/*===========================================================================*/
// General purpose buffer for reading results
static uint8_t rxbuff[RXBUFF_SIZE]  __attribute__ ((section(".bufram")));
static char txbuff[TXBUFF_SIZE]  __attribute__ ((section(".bufram")));
static char linebuff[LINEBUFF_SIZE]  __attribute__ ((section(".bufram")));

static char dbgChar;

static void notify(io_queue_t *qp) { (void)qp; }
static INPUTQUEUE_DECL(rxq, rxbuff, RXBUFF_SIZE, notify, NULL);

static uint8_t ip_addr[] = {0,0,0,0};
static uint8_t ip_gateway[] = {0,0,0,0};
static uint8_t ip_netmask[] = {0,0,0,0};

// Threads
static THD_FUNCTION(rx_process_thread, arg);
static THD_WORKING_AREA(rx_process_thread_wa, DEFS_THD_ESPRXDBG_WA_SIZE);
static thread_t *process_tp;

static char buf[16];
/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg_1 = {
  txend1,
  txend2,
  rxend,
  rxchar,
  rxerr,
  115200,
  0,
  0,
  0
};

typedef enum {
    WIFI_RESET = 0,
    WIFI_INITIALIZED,
    WIFI_AP_CONNECTED
} wifiStatus_t;

static const espReturn_t returnValues[] = {
   { ESP_RET_SENT,              "SEND OK\r\n" },
   { ESP_RET_SENTFAIL,          "SEND FAIL\r\n" },
   { ESP_RET_ERROR,             "ERROR\r\n" },
   { ESP_RET_UNLINK,            "Unlink\r\n" },
   { ESP_RET_LINKED,            "Linked\r\n" },
   { ESP_RET_CLOSED,            ",CLOSED\r\n" },
   { ESP_RET_CONNECT,           ",CONNECT\r\n" },
   { ESP_RET_WIFI_GOT_IP,       "WIFI GOT IP\r\n" },
   { ESP_RET_WIFI_CONNECT,      "WIFI CONNECTED\r\n" },
   { ESP_RET_WIFI_DISCONNECT,   "WIFI DISCONNECT\r\n" },
   { ESP_RET_READY,             "ready\r\n" },
   { ESP_RET_IPD,               "+IPD,"  },
   { ESP_RET_OK,                "OK\r\n" },
   { ESP_RET_IP,                "+CIPSTA:ip:" },
   { ESP_RET_GATE,              "+CIPSTA:gateway:" },
   { ESP_RET_NETMA,             "+CIPSTA:netmask:" },
   { ESP_RET_NONE,              "\r\n" },
};
#define NUM_RESPONSES sizeof(returnValues)/sizeof(espReturn_t)

static wifiStatus_t espStatus = WIFI_RESET;
/*===========================================================================*/
/* USART callbacks                                                           */
/*===========================================================================*/

/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {
  (void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {
  (void)uartp;
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {
  (void)uartp;
  (void)e;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
  (void)uartp;
  chSysLockFromISR();
  chIQPutI(&rxq, (uint8_t)c);
  dbgChar = c;
  if(process_tp) chEvtSignalI(process_tp, (eventmask_t) 1);
  chSysUnlockFromISR();
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
  (void)uartp;
}

/*===========================================================================*/
/* RX Buffer functions                                                       */
/*===========================================================================*/
/**
 * @brief      RX thread, gets event from ISR when char is received
 * @details    Debug only
 *
 * @param[in]  <unnamed>  { parameter_description }
 * @param[in]  <unnamed>  { parameter_description }
 */
static THD_FUNCTION(rx_process_thread, arg) {
  (void)arg;

  chRegSetThreadName(DEFS_THD_ESPRXDBG_NAME);
  process_tp = chThdGetSelfX();

  for(;;)
  {
    chEvtWaitAny((eventmask_t) 1);
    DBG("%c", dbgChar);
  }
}

/*===========================================================================*/
/* Module static functions.                                                  */
/*===========================================================================*/
/**
 * @brief      Returns one character from rx buffer
 *
 * @param[in]  timeout  The to wait maximal
 *
 * @return     character from input buffer or -1 if buffer is empty and timeout
 * has occured
 */
static int rxget (uint32_t timeout)
{
  uint32_t tcnt = 0;

  // if(timeout > 0) chThdSleepMicroseconds(timeout);
  chSysLock();

  while ( (tcnt++ < timeout) && chIQIsEmptyI(&rxq) ) 
  {
    chSysUnlock();
    chThdSleepMicroseconds(1);  
    chSysLock();
  }
  if(chIQIsEmptyI(&rxq))
  {
    chSysUnlock();
    return -1;
  }

  chSysUnlock();
  return chIQGet(&rxq);
}

/**
 * @brief      Reat input buffer until resp and block for timeout ms
 *
 * @param[in]  resp     The resp
 * @param[in]  timeout  The timeout
 *
 * @return     true if response was found in given timeout
 */
bool esp8266ReadUntil(const char * resp, int timeout)
{
    char c = 0;
    int ret;
    int index = 0;
    int targetLength = strlen(resp);
    int ctr = 0;

    if(resp == NULL) return true;

    memset(linebuff, 0, LINEBUFF_SIZE);

    ret = rxget(timeout);
    for(; ret >= 0; )
    {
      c = (char)ret;
      linebuff[ctr++] = c;
      ctr%=LINEBUFF_SIZE;
      if (c != resp[index])
      {
        index = 0;
      }
      if (c == resp[index]){
         if(++index >= targetLength){
              return true;
         }
      }
      ret = rxget(timeout);
    }
    return false;
}

/**
 * @brief      Send esp command and wait for response
 *
 * @param[in]  cmd       The command
 * @param[in]  rsp       The response
 * @param[in]  cmddelay  time to wait after sending
 *
 * @return     true if success, false if timeout
 */
bool esp8266Cmd(const char * cmd, const char * rsp, int cmddelay)
{
  // memset(txbuff, 0, TXBUFF_SIZE);
  chsnprintf(txbuff, TXBUFF_SIZE, "%s\r\n\0", cmd);
  // Send the command to the wifi chip 
  DBG(">>%s\r\n", txbuff);
  while(UARTD2.txstate == UART_TX_ACTIVE) chThdSleepMilliseconds(1);
  uartStartSend(&UARTD2, strlen(cmd)+2, txbuff); //+2 for \r\n
  if (cmddelay > 0) 
  {
    chThdSleepMilliseconds(cmddelay);
  }

  if(rsp != NULL) return esp8266ReadUntil(rsp, READ_UNTIL_TIMEOUT_US);
  return true;
}

/**
 * @brief      Send data string to the esp
 *
 * @param[in]  data  The data
 */
static void esp8266Data(const void * data, int msglen)
{
  while(UARTD2.txstate == UART_TX_ACTIVE) chThdSleepMilliseconds(1);
  memcpy(txbuff, data, msglen);
  uartStartSend(&UARTD2, msglen, txbuff);
}

/**
 * @brief      Set WiFi mode
 *
 * @param[in]  mode  The mode
 *
 * @return     true if success
 */
bool esp8266SetMode(int mode)
{
  chsnprintf(txbuff, TXBUFF_SIZE, "AT+CWMODE=%d", mode);
  return (esp8266Cmd(txbuff, "OK", 100));
}

/**
 * @brief      Read the rx buffer until one of the given retvals
 *
 * @param[in]  retvals  The retvals
 * @param      buffer   The buffer
 * @param      bufsiz   The bufsiz
 * @param[in]  timeout  The timeout
 *
 * @return     The response found in the input buffer
 */
static uint32_t esp8266ReadSwitch(uint32_t retvals, char * buffer, int * bufsiz, int timeout)
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
  c = rxget(timeout);
  while(c >= 0)
  {
    if (numstored < *bufsiz)
    {
      buffer[numstored] = c;
      numstored++;

      // Evaluate if this belongs to the
      // list of return values
      for(x = 0; x < numlen; x++)
      {
        if (c != returnValues[x].retstr[index[x]])
        {
          index[x] = 0;
        }
        if (c == returnValues[x].retstr[index[x]])
        {
          index[x]++;
          if(index[x] >= lens[x])
          {
            if(retvals & returnValues[x].retval)
            {
              buffer[numstored] = 0;
              *bufsiz = numstored;
              DBG("\r\n**Got [%s], returning %d\r\n", buffer, returnValues[x].retval);
              return returnValues[x].retval;
            }
          }
        }
      }
      // Reset the buffer on a newline or linefeed
      if ((numstored >= 2) && (buffer[numstored-2] == '\r') && (buffer[numstored-1] == '\n'))
      {
        numstored = 0;
      }

    }
    else
    {
      break;
    }
    c = rxget(timeout);
  }

  buffer[numstored] = 0;
  *bufsiz = numstored;
  DBG("\r\n**Got [%s], returning with -1\r\n", buffer);
  return -1;
}

/**
 * @brief      Reads the ip address from the line buffer
 */
static void parseIP(void)
{
  char* ptr;
  int pos;
  ptr = linebuff;
  if(ptr == NULL) return;
  else
  {
    pos = (int)(ptr - linebuff);
    memset(buf,0,16);
    pos+=1; // goto start of ip string
    for(int i=0; linebuff[pos]!='\"'; pos++) buf[i++] = linebuff[pos]; //copy ip string
    str2ip(buf,ip_addr);
  }
}

/**
 * @brief      reads the gateway from the linebuffer
 */
static void parseGateway(void)
{
  char* ptr;
  int pos;
  ptr = linebuff;
  if(ptr == NULL) return;
  else
  {
    pos = (int)(ptr - linebuff);
    memset(buf,0,16);
    pos+=1; // goto start of ip string
    for(int i=0; linebuff[pos]!='\"'; pos++) buf[i++] = linebuff[pos]; //copy ip string
    str2ip(buf,ip_gateway);
  }
}

/**
 * @brief      reads the netmask from the linebuffer
 */
static void parseNetmask(void)
{
  char* ptr;
  int pos;
  ptr = linebuff;
  if(ptr == NULL) return;
  else
  {
    pos = (int)(ptr - linebuff);
    memset(buf,0,16);
    pos+=1; // goto start of ip string
    for(int i=0; linebuff[pos]!='\"'; pos++) buf[i++] = linebuff[pos]; //copy ip string
    str2ip(buf,ip_netmask);
  }
}

/*===========================================================================*/
/* Module public functions.                                                  */
/*===========================================================================*/
/**
 * @brief      Starts the esp comm thread
 */
void espStart(void) {
  //PD pin init
  palSetPadMode(ESP_PD_PORT, ESP_PD_PIN, PAL_MODE_OUTPUT_PUSHPULL); /* PD4 */
  palSetPad(ESP_PD_PORT, ESP_PD_PIN);

  (void)chThdCreateStatic(rx_process_thread_wa, 
    sizeof(rx_process_thread_wa), 
    NORMALPRIO, 
    rx_process_thread, 
    NULL);  

  /*
  * Activates the serial driver 2 using the driver default configuration.
  * PA2(TX) and PA3(RX) are routed to USART2.
  */
  // sdStart(&SD2, &sd2conf);
  uartStart(&UARTD2, &uart_cfg_1);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));
}

/**
 * @brief      Init the esp8266 
 *
 * @return     ESP_RET_OK of all went well, error code if else
 */
int espInit(void) {
  DBG("Starting esp init...\r\n");
  

  if (!esp8266Cmd("AT+RST", "ready", 1000))
  {
      DBG("Failed to reset ESP8266!\r\n");
      return ESP_RET_INVAL;
  }
  else
  {
    espStatus = WIFI_INITIALIZED;
    DBG("\r\nESP8266 Initialized\r\n");
  }

  if(esp8266SetMode(WIFI_MODE_STA))
  {
    DBG("ESP8266 Mode set to %d\r\n", WIFI_MODE_STA);
  }

  if (!esp8266Cmd("AT+CIPMUX=1", "OK", 500))
  {
      DBG("Failed to set ESP8266 MUX!\r\n");
      return ESP_RET_INVAL;
  }

  return ESP_RET_OK;
}

/**
 * @brief      Checks if the ESP module has an IP
 *
 * @return     true if IP is valid
 */
bool espHasIP(void) 
{
  esp8266Cmd("AT+CIPSTA?", NULL, 100); 
  return espStatus == WIFI_AP_CONNECTED ? true:false;
}

/**
 * @brief      Connect to hotspot
 *
 * @param[in]  ssid      The ssid
 * @param[in]  password  The password
 *
 * @return     ESP_RET value
 */
int espConnectAP(const char *ssid, const char *password)
{
  char buf[100];

  DBG("ESP8266 Joining AP (%s) ... ", ssid);

  chsnprintf(buf, 100, "AT+CWJAP=\"%s\",\"%s\"", ssid, password);

  if (!esp8266Cmd(buf, "OK", 1000))
  {
      DBG("Failed to connect AP!\r\n");
      return ESP_RET_INVAL;
  }
  DBG("AP connect success\r\n");
  return ESP_RET_OK;
}

/**
 * @brief      Sends the WPS Start connect command
 */
void espStartWPS(void) {
  esp8266Cmd("AT+WPS", "OK", 1000);
}

/**
 * @brief      Returns true if the rx buffer has data
 *
 * @return     true if buffer has data in it
 */
bool esp8266HasData(void)
{
  bool ret;
  chSysLock();
  ret = chIQIsEmptyI(&rxq);
  chSysUnlock();
  return !ret;
}

/**
 * @brief      Start a server channel
 *
 * @param[in]  channel  The channel
 * @param[in]  type     The type
 * @param[in]  port     The port
 *
 * @return     ESP_RET value
 */
int esp8266Server(int channel, int type, uint16_t port)
{
    int mode = 0;
    (void)channel;
    // TODO: Esp8266 firmware can not yet start a UDP
    // server, so type here is always TCP
    if (type == ESP_UDP) return -1;

    chsnprintf(txbuff, TXBUFF_SIZE, "AT+CIPSERVER=%d,%d\r\n",
        mode, port);

    return esp8266Cmd(txbuff, "OK", 100);       
}

/**
 * @brief      Connect to a server socket
 *
 * @param[in]  channel  The channel
 * @param[in]  ip       server ip
 * @param[in]  port     The port
 * @param[in]  type     The type
 *
 * @return     ESP_RET value
 */
int esp8266Connect(int channel, const char * ip, uint16_t port, int type)
{
  // Simply send the data over the channel.
  chsnprintf(txbuff, TXBUFF_SIZE, "AT+CIPSTART=%d,\"%s\",\"%s\",%d\r\n",
      channel,
      type == ESP_TCP ? "TCP":"UDP",
      ip,
      port);

  // For my particular firmware AT+CIPSTART returns OK
  if(esp8266Cmd(txbuff, "OK" , 300)) return ESP_RET_OK;
  return ESP_RET_ERROR;
}

/**
 * @brief      Close selected channel
 *
 * @param[in]  channel  The channel
 *
 * @return     true if succeeded
 */
bool esp8266Disconnect(int channel)
{
  chsnprintf(txbuff, TXBUFF_SIZE, "AT+CIPCLOSE=%d\r\n", channel);
  return (esp8266Cmd(txbuff, "OK", 100));
}

/**
 * @brief      Send a line of data over a given channel
 *
 * @param[in]  channel  The channel
 * @param[in]  str      The string
 *
 * @return     true if succeeded
 */
bool esp8266SendLine(int channel, const char * str)
{
  int datatosend = 0;

  if (str)
    datatosend = strlen(str) + 2;
  else
    datatosend = 2; // \r\n (empty lines)

  chsnprintf(txbuff, TXBUFF_SIZE, "AT+CIPSEND=%d,%d\r\n",
             channel,
             datatosend);
  // Wait untill the prompt
  if (esp8266Cmd(txbuff, ">", 10))
  {
    DBG("\r\n>>Got the prompt! Sending rest of data!\r\n");
    chsnprintf(txbuff, TXBUFF_SIZE, "%s\r\n",(const char*)str);
    esp8266Data(txbuff, strlen(txbuff));

    return esp8266ReadUntil("SEND OK\r\n", READ_UNTIL_TIMEOUT_US);
  }

  return false;
}

/**
 * @brief      Send the start transmission command
 *
 * @param[in]  channel     The channel
 * @param[in]  datatosend  number of data bytes to send
 *
 * @return     true if succeeded
 */
bool esp8266SendHeader(int channel, int datatosend)
{
  chsnprintf(txbuff, TXBUFF_SIZE, "AT+CIPSEND=%d,%d\r\n",
             channel,
             datatosend);

  // Wait untill the prompt
  if (esp8266Cmd(txbuff, ">", 10))
  {
    DBG(">>Got the command prompt! ... send the rest of the data!\r\n");
    return true;
  }

  return false;
}

/**
 * @brief      Send block of data
 *
 * @param[in]  data       The data
 * @param[in]  waitforok  wait for the OK from the esp8266
 *
 * @return     true if succeeded
 */
bool esp8266Send(const void * data, int msglen, bool waitforok)
{
  // send data
  esp8266Data(data, msglen);
  
  if (waitforok)
  {
      return esp8266ReadUntil("SEND OK", 100000);
  }
  return true;
}

/**
 * @brief      Reads the rx for possible data
 *
 * @param      channel  out: channel, where the data occured
 * @param      param    out: number of bytes received
 * @param[in]  timeout  The timeout
 *
 * @return     return status of the esp ESP_RET_xxx
 */
uint32_t esp8266ReadRespHeader(int * channel, int * numbytes, int timeout)
{
  char *p = NULL;
  int numread = 0;
  int bufsiz = LINEBUFF_SIZE;
  uint32_t retval;

  // Discard data until we receive part of the header
  DBG(">>Waiting for message header ...\r\n");
  // Read loop until we have a status
  retval = esp8266ReadSwitch(ESP_RET_ERROR |
                             ESP_RET_CONNECT |
                             ESP_RET_CLOSED |
                             ESP_RET_SENT |
                             ESP_RET_IPD |
                             ESP_RET_IP |
                             ESP_RET_GATE |
                             ESP_RET_NETMA |
                             ESP_RET_WIFI_GOT_IP |
                             ESP_RET_WIFI_CONNECT |
                             ESP_RET_WIFI_DISCONNECT
                             , linebuff, &bufsiz, timeout);
  DBG(">>Retval = %d\r\n", retval);
  if(retval == ESP_RET_IPD)
  {
      DBG(">>Read the +IPD, reading message length and channel ..\r\n");
      // Read header information (up until the ":")
      // +IPD,0,5:asdf
      memset(linebuff, 0, LINEBUFF_SIZE);
      if ((numread = esp8266ReadUntil(":", READ_UNTIL_TIMEOUT_US)) > 0)
      {
          // Parse header information for
          // Channel and number of bytes
          p = strtok(linebuff,":,");
          if (p) *channel = atoi(p);
          p = strtok(NULL, ":,");
          if (p) *numbytes = atoi(p);
          DBG(">> Channel = %d, bytestoread = %d\r\n", *channel, *numbytes);
          // chThdSleepMilliseconds(1000);
      }
  }

  if ((retval == ESP_RET_CONNECT) ||
      (retval == ESP_RET_CLOSED))
  {
      p = strtok(linebuff, ",");
      if (p) *channel = atoi(p);
      DBG(">>Read closed/connect status on channel %d.. \r\n", *channel);
  }
  if (retval == ESP_RET_IP)
  {
    if ((numread = esp8266ReadUntil("\n", READ_UNTIL_TIMEOUT_US)) > 0)
    {
      parseIP();
      DBG("IP: %d.%d.%d.%d\r\n", ip_addr[0],ip_addr[2],ip_addr[2],ip_addr[3]);
      if(ip_addr[0] != 0) espStatus = WIFI_AP_CONNECTED;
      else espStatus = WIFI_INITIALIZED;
    }
  }
  if (retval == ESP_RET_GATE)
  {
    if ((numread = esp8266ReadUntil("\n", READ_UNTIL_TIMEOUT_US)) > 0)
    {
      parseGateway();
      DBG("GW: %d.%d.%d.%d\r\n", ip_gateway[0],ip_gateway[2],ip_gateway[2],ip_gateway[3]);
    }
  }
  if (retval == ESP_RET_NETMA)
  {
    if ((numread = esp8266ReadUntil("\n", READ_UNTIL_TIMEOUT_US)) > 0)
    {
      parseNetmask();
      DBG("NM: %d.%d.%d.%d\r\n", ip_netmask[0],ip_netmask[2],ip_netmask[2],ip_netmask[3]);
    }
  }
  if (retval == ESP_RET_WIFI_DISCONNECT)
  {
    espStatus = WIFI_AP_CONNECTED;
  }
  if (retval == ESP_RET_WIFI_CONNECT)
  {
    espStatus = WIFI_AP_CONNECTED;
  }
  if (retval == ESP_RET_WIFI_GOT_IP)
  {
    espStatus = WIFI_AP_CONNECTED;
  }

  return retval;
}

/**
 * @brief      Read received TCP/UDP data
 *
 * @param      buffer       The buffer to store the data
 * @param[in]  bytestoread  The bytestoread num bytes to read
 *
 * @return     number of bytes read
 */
int esp8266Read(char * buffer, int bytestoread)
{
  int numread = 0;
  int c;

  do {
    //c = sdGetTimeout((SerialDriver *) usart, READ_UNTIL_TIMEOUT_US);
    c = rxget(0);
    if (c >= 0)
    {
        buffer[numread] = c; 
        numread ++;
    }else
      break;
  }while(numread < bytestoread);

#if DEFS_ESP_DBG == TRUE
  if (numread > 0)
  { 
    DBG("\r\n>>Read %d bytes ... dumping data\r\n", numread);
    hexdump(dbgstrm, buffer, numread);
  }
#endif
  
  return numread;
}

/**
 * @brief      Read single byte from input buffer
 *
 * @param[in]  timeout  The timeout
 *
 * @return     { description_of_the_return_value }
 */
int esp8266Get(int timeout)
{
  return rxget(timeout);
}

/**
 * @brief      Controlls the ESP sleep pin
 *
 * @param[in]  sl    true if set to sleep, false if to wake up
 */
void esp8266Sleep(bool sl)
{
  if(sl) palClearPad(ESP_PD_PORT, ESP_PD_PIN);
  else 
  {
    palSetPad(ESP_PD_PORT, ESP_PD_PIN);
    chThdSleepMilliseconds(300);
    // resend cipmux
    while(channelSetCipmux() == false);
    // reset status to re check IP
    espStatus = WIFI_AP_CONNECTED;
  }
}
/** @} */
