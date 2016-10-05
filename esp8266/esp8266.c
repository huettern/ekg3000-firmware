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

#define READ_TIMEOUT 1000 // uart read timeout on 1000 ticks
#define WRITE_TIMEOUT 1000 // uart write timeout on 1000 ticks
// General purpose buffer for reading results
static char rxbuff[RXBUFF_SIZE]  __attribute__ ((section(".bufram")));
static char txbuff[TXBUFF_SIZE]  __attribute__ ((section(".bufram")));

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

  if (serial_rx_write_pos == RXBUFF_SIZE) {
    serial_rx_write_pos = 0;
  }

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
    int c = 0;
    int index = 0;
    int targetLength = strlen(resp);
    memset(rxbuff, 0, RXBUFF_SIZE);
    int rxbufSize = 0;

    for(;;)
    {
      // c = sdGetTimeout((SerialDriver *) usart, timeout);
      if( (c == Q_TIMEOUT) || (c == Q_RESET) ) 
        {
          if(c == Q_TIMEOUT) DBG("Q_TIMEOUT");
          if(c == Q_RESET) DBG("Q_RESET");
          
          return false;
        }
      rxbuff[rxbufSize++]=c;
      DBG("%c", c);
      if (c != resp[index])
            index = 0;

      if (c == resp[index]){
         if(++index >= targetLength){
              return true;
         }
      }
    }
          DBG("FDSA");
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

  // return esp8266ReadUntil(rsp, READ_TIMEOUT);
  return false;
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

int espInit(void) {

  DBG("Starting esp init...\r\n");
  

  if (!esp8266Cmd("AT+RST", "ready", 2000))
  {
      DBG("Failed to reset ESP8266!\r\n");
      return -1;
  }
  else
  {
    DBG("\r\nESP8266 Initialized\r\n");
  }
    

  // // DBG("ESP8266 Firmware Version [%s]\r\n", esp8266GetFirmwareVersion());

  // if(esp8266SetMode(WIFI_MODE_STA))
  // {
  //   DBG("ESP8266 Mode set to %d\r\n", WIFI_MODE_STA);
  // }

  return -1;
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

  if (!esp8266Cmd("AT+CIPSTA?", "OK", 5000))
  {
      DBG("Failed to get ESP8266 IP STATUS!\r\n");
      return false;
  }
  // Now parse the received data
  memset(ip_addr,0,4);
  memset(ip_gateway,0,4);
  memset(ip_netmask,0,4);

  // get IP
  ptr = strstr(rxbuff,"ip");
  if(ptr == NULL) return false;
  else
  {
    pos = (int)(ptr - rxbuff);
    memset(buf,0,16);
    pos+=4; // goto start of ip string
    for(int i=0; rxbuff[pos]!='\"'; pos++) buf[i++] = rxbuff[pos]; //copy ip string
    DBG("extracted string:%s\r\n",buf);
    str2ip(buf,ip_addr);
  }

  // get gateway
  ptr = strstr(rxbuff,"gateway");
  if(ptr == NULL) return false;
  else
  {
    pos = (int)(ptr - rxbuff);
    memset(buf,0,16);
    pos+=4; // goto start of ip string
    for(int i=0; rxbuff[pos]!='\"'; pos++) buf[i++] = rxbuff[pos]; //copy ip string
    DBG("extracted string:%s\r\n",buf);
    str2ip(buf,ip_gateway);
  }

  // get subnet
  ptr = strstr(rxbuff,"netmask");
  if(ptr == NULL) return false;
  else
  {
    pos = (int)(ptr - rxbuff);
    memset(buf,0,16);
    pos+=4; // goto start of ip string
    for(int i=0; rxbuff[pos]!='\"'; pos++) buf[i++] = rxbuff[pos]; //copy ip string
    DBG("extracted string:%s\r\n",buf);
    str2ip(buf,ip_netmask);
  }

  DBG("IP: %d.%d.%d.%d\r\n", ip_addr[0],ip_addr[2],ip_addr[2],ip_addr[3]);
  DBG("GW: %d.%d.%d.%d\r\n", ip_gateway[0],ip_gateway[2],ip_gateway[2],ip_gateway[3]);
  DBG("NM: %d.%d.%d.%d\r\n", ip_netmask[0],ip_netmask[2],ip_netmask[2],ip_netmask[3]);

  return true;
}



static THD_FUNCTION(rx_process_thread, arg) {
  (void)arg;

  chRegSetThreadName("uartcomm process");

  process_tp = chThdGetSelfX();

  for(;;)
  {
    chEvtWaitAny((eventmask_t) 1);

    while (serial_rx_read_pos != serial_rx_write_pos) {
      DBG("%c", rxbuff[serial_rx_read_pos++]);
      if (serial_rx_read_pos == RXBUFF_SIZE) {
        serial_rx_read_pos = 0;
      }
    }
  }

}



void espTerm(char* str) {
  DBG(">%s\r\n",str);
  espRead();
}

void espRead() {
  char buf[200];
  DBG("<%s\r\n",buf);
}
