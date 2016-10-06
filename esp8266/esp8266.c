#include "esp8266.h"
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "util.h"
#include "wifichannel.h"

#include "usbcfg.h"


#include <stdlib.h>
#include <string.h>

// UART device to use
#define uart &UARTD2

static BaseSequentialStream * dbgstrm = bssusb;

#define DBG(X, ...)    chprintf(dbgstrm, X, ##__VA_ARGS__ )
// #define DBG(X, ...)

#define RXBUFF_SIZE 2048
#define TXBUFF_SIZE 200
#define LINEBUFF_SIZE 2048

#define READ_TIMEOUT 1000 // uart read timeout on 1000 ticks
#define WRITE_TIMEOUT 1000 // uart write timeout on 1000 ticks
// General purpose buffer for reading results
static char rxbuff[RXBUFF_SIZE]  __attribute__ ((section(".bufram")));
static char txbuff[TXBUFF_SIZE]  __attribute__ ((section(".bufram")));
static char linebuff[LINEBUFF_SIZE]  __attribute__ ((section(".bufram")));

static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;

static uint8_t ip_addr[] = {0,0,0,0};
static uint8_t ip_gateway[] = {0,0,0,0};
static uint8_t ip_netmask[] = {0,0,0,0};

static void txend1(UARTDriver *uartp);
static void txend2(UARTDriver *uartp);
static void rxerr(UARTDriver *uartp, uartflags_t e);
static void rxchar(UARTDriver *uartp, uint16_t c);
static void rxend(UARTDriver *uartp);

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

// Threads
static THD_FUNCTION(rx_process_thread, arg);
static THD_WORKING_AREA(rx_process_thread_wa, 4096);
static thread_t *process_tp;


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
  rxbuff[serial_rx_write_pos++] = c;

  serial_rx_write_pos %= RXBUFF_SIZE;

  chSysLockFromISR();
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
 *
 * @param[in]  <unnamed>  { parameter_description }
 * @param[in]  <unnamed>  { parameter_description }
 */
static THD_FUNCTION(rx_process_thread, arg) {
  (void)arg;

  chRegSetThreadName("uartcomm process");

  process_tp = chThdGetSelfX();

  for(;;)
  {
    chEvtWaitAny((eventmask_t) 1);

    DBG("%c", rxbuff[serial_rx_write_pos-1]);

    // while (serial_rx_read_pos != serial_rx_write_pos) {
    //   DBG("%c", rxbuff[serial_rx_read_pos++]);
    //   serial_rx_read_pos %= RXBUFF_SIZE;
    // }
  }

}

/*===========================================================================*/
/* Module static functions.                                                  */
/*===========================================================================*/
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
    int index = 0;
    int targetLength = strlen(resp);
    int ctr = 0;
    memset(linebuff, 0, LINEBUFF_SIZE);

    for(; serial_rx_read_pos < serial_rx_write_pos; serial_rx_read_pos++)
    {
      c = rxbuff[serial_rx_read_pos];
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
  chsnprintf(txbuff, TXBUFF_SIZE, "%s\r\n", cmd);
  // Send the command to the wifi chip 
  DBG(">>%s\r\n", txbuff);
  uartStartSend(&UARTD2, strlen(cmd)+2, txbuff);
  if (cmddelay > 0) 
  {
    chThdSleepMilliseconds(cmddelay);
  }

  return esp8266ReadUntil(rsp, READ_TIMEOUT);
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



/*===========================================================================*/
/* Module public functions.                                                  */
/*===========================================================================*/
/**
 * @brief      Starts the esp comm thread
 */
void espStart(void) {
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
 * @return     { description_of_the_return_value }
 */
int espInit(void) {
  DBG("Starting esp init...\r\n");
  

  if (!esp8266Cmd("AT+RST", "ready", 1500))
  {
      DBG("Failed to reset ESP8266!\r\n");
      return ESP_RET_INVAL;
  }
  else
  {
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
  char* ptr;
  int pos;
  char buf[16];

  if (!esp8266Cmd("AT+CIPSTA?", "OK", 1000))
  {
      DBG("Failed to get ESP8266 IP STATUS!\r\n");
      return false;
  }
  DBG("Got ESP8266 IP STATUS!\r\n");
  
  // Now parse the received data
  memset(ip_addr,0,4);
  memset(ip_gateway,0,4);
  memset(ip_netmask,0,4);

  // get IP
  ptr = strstr(linebuff,"ip");
  if(ptr == NULL) return false;
  else
  {
    pos = (int)(ptr - linebuff);
    memset(buf,0,16);
    pos+=4; // goto start of ip string
    for(int i=0; linebuff[pos]!='\"'; pos++) buf[i++] = linebuff[pos]; //copy ip string
    str2ip(buf,ip_addr);
  }

  // get gateway
  ptr = strstr(linebuff,"gateway");
  if(ptr == NULL) return false;
  else
  {
    pos = (int)(ptr - linebuff);
    memset(buf,0,16);
    pos+=9; // goto start of ip string
    for(int i=0; linebuff[pos]!='\"'; pos++) buf[i++] = linebuff[pos]; //copy ip string
    str2ip(buf,ip_gateway);
  }

  // get subnet
  ptr = strstr(linebuff,"netmask");
  if(ptr == NULL) return false;
  else
  {
    pos = (int)(ptr - linebuff);
    memset(buf,0,16);
    pos+=9; // goto start of ip string
    for(int i=0; linebuff[pos]!='\"'; pos++) buf[i++] = linebuff[pos]; //copy ip string
    str2ip(buf,ip_netmask);
  }

  DBG("IP: %d.%d.%d.%d\r\n", ip_addr[0],ip_addr[2],ip_addr[2],ip_addr[3]);
  DBG("GW: %d.%d.%d.%d\r\n", ip_gateway[0],ip_gateway[2],ip_gateway[2],ip_gateway[3]);
  DBG("NM: %d.%d.%d.%d\r\n", ip_netmask[0],ip_netmask[2],ip_netmask[2],ip_netmask[3]);

  if(ip_addr[0] != 0) return true;
  return false;
}

/**
 * @brief      Connect to hotspot
 *
 * @param[in]  ssid      The ssid
 * @param[in]  password  The password
 *
 * @return     { description_of_the_return_value }
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


void espTerm(char* str) {
  DBG(">%s\r\n",str);
  espRead();
}

void espRead() {
  char buf[200];
  DBG("<%s\r\n",buf);
}
