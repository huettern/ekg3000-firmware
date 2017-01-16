/*
   EKG3000 - Copyright (C) 2016 FHNW Project 3 Team 2
 */

/**
 * @file       wifiapp.c
 * @brief      WiFi application thread for MQTT communication and data transfer
 * @details    
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       1 December 2016
 * 
 *
 * @addtogroup WIFIAPP
 * @brief Wifi application module
 * @{
 */

#include "wifiapp.h"

#include "ch.h"
#include "chprintf.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "wifichannel.h"
#include "wifisocket.h"
#include "usbcfg.h"
#include "esp8266.h"
#include "util.h"
#include "libemqtt.h"
#include "defs.h"
#include "ui.h"

#include "ff.h"

/*===========================================================================*/
/* settings                                                                */
/*===========================================================================*/
/*
 * defines the thread interval in ms
 */
#define THD_INTERVAL 1000

/**
 * How many THD_INTERVAL to wait before connecting to default AP
 */
#define AP_CONNECT_DELAY 30

/**
 * The maximum value length of a value in the input file in bulk transfer mode.
 * Used to calculate the optimal packet size. 
 * CHECK WITH PRINTF FOMRAT SPECIFIER IN MODULE analog.c
 * %1.5f => 7 characters
 */
#define BLK_TX_VALUE_LENGHT 7

/**
 * path to the wifi config file on sd card
 * the config file should have following structure
 * <ap name string>
 * <ssid string>
 */
#define WIFI_CFG_PATH _T("/WC.TXT")
#define LINE_BUF_SIZE 34

/**
 * MQTT Broker IP address
 */
#define MQTT_SERVER_ADDRESS "46.126.176.250"
/**
 * MQTT Broker Port number
 */
#define MQTT_SERVER_PORT 4283
/**
 * MQTT client identification
 */
#define MQTT_CLIENT_ID "ekg3000proto"
/**
 * MQTT topic for publishing
 */
#define MQTT_TOPIC "ekg3000/proto"

static THD_WORKING_AREA(waWifi, DEFS_THD_WIFIAPP_WA_SIZE);

static BaseSequentialStream * dbgstrm = bssusb;
#if DEFS_WIFIAPP_DBG == TRUE
  #define DBG(X, ...)    chprintf(dbgstrm, X, ##__VA_ARGS__ )
#else
  #define DBG(X, ...)
#endif

/*===========================================================================*/
/* Module local function prototypes                                          */
/*===========================================================================*/
static void fWiFiStateMachine(void);
static bool wifiAppInit(void);
static void initMQTT(void);

// MQTT related
static int send_packet(void* socket_info, const void* buf, unsigned int count);
static int init_socket(mqtt_broker_handle_t* broker, const char* hostname, short port);
static int close_socket(mqtt_broker_handle_t* broker);
static int read_packet(int timeout);

static bool checkPubackQoS1(uint16_t msg_id);
static void publishQoS1 (mqtt_broker_handle_t* broker, const char* topic, const char* msg);

static bool initBlkTx(void);
static void runBlkTx(void);
static void stopBlkTx (void);

/*===========================================================================*/
/* Module local data                                                         */
/*===========================================================================*/
static thread_t *tp = NULL;
static wifi_state state = 0;
static mutex_t mtx;

static uint8_t apConDlyCtr = 0;

static bool blSendDataRdy = false;
static bool blDefaultAPAvail = false;
static bool blMQTTRunning = false;
static bool blStartWPS = false;
static bool blStartBulkTransfer = false;
static bool blGotoSleep = false;

#define TXMSG_SIZE 1400
static char txmsg[TXMSG_SIZE];

#define WIFI_SSID_PW_SIZE 25
static char wifi_ssid[WIFI_SSID_PW_SIZE];
static char wifi_pw[WIFI_SSID_PW_SIZE];

static char linebuf[LINE_BUF_SIZE];

static char* fname;
static FIL fp;
static uint16_t samplerate;

// MQTT related
#define RCVBUFSIZE 1024
uint8_t packet_buffer[RCVBUFSIZE];
static mqtt_broker_handle_t broker;
int socket_id;

#define BLK_TX_VALS_PER_BLOCK (TXMSG_SIZE-8)/(BLK_TX_VALUE_LENGHT+1)

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/
/**
 * @brief      handles wifi statemachine, called periodically
 */
static void fWiFiStateMachine(void)
{
  int ret;
  while(!chMtxTryLock(&mtx)) chThdSleepMilliseconds(5);
  // chMtxLock(&mtx);
  switch(state)
  {
    case WIFI_APP_DEINIT:
      if(wifiAppInit() == true) state=WIFI_DEINIT;
      else if(blGotoSleep) state = WIFI_SLEEP;
      break;
    case WIFI_DEINIT:
      if(wifiInit() == ESP_RET_OK) state=WIFI_INITIALIZED;
      break;
    case WIFI_INITIALIZED:
      // check if ip gained
      if(wifiHasIP()) 
      {
        #ifdef DEFS_TEST_WIFI_RANGE
          state=WIFI_TEST_RANGE;
          break;
        #endif
        #ifdef DEFS_TEST_WIFI_FILE
          state=WIFI_TEST_FILE;
          break;
        #endif
        // goto sleep mode if no request is pending
        if(!blSendDataRdy && !blStartBulkTransfer)
        {
          blGotoSleep = true;
          state=WIFI_SLEEP;
          break; 
        }
        else
        {
          state = WIFI_CONNECTED;
        }
      }
      // check if WPS connection start
      if(blStartWPS == true) state=WIFI_WPS;
      // check if connect to default AP
      // else if (++apConDlyCtr >= AP_CONNECT_DELAY)
      // {
      //   wifiConnectAP(wifi_ssid, wifi_pw);
      //   blDefaultAPAvail = false;
      //   apConDlyCtr = 0;
      // }
      else if(blGotoSleep) state = WIFI_SLEEP;
      break;
    case WIFI_CONNECTED:
      if(!blMQTTRunning) {initMQTT(); break;}
      // checl switch state condition
      if(blStartBulkTransfer) state = WIFI_BULK_TRANSFER;
      else if(blSendDataRdy) state=WIFI_START_TRANSMISSION;
      else if(blGotoSleep) state = WIFI_SLEEP;
      break;
    case WIFI_START_TRANSMISSION:
      DBG("publish...");
      ret = mqtt_publish(&broker, MQTT_TOPIC, txmsg, 0);
      DBG(" ret=%d\r\n", ret);
      blSendDataRdy = false;
      state=WIFI_CONNECTED;
      break;
    case WIFI_WPS:
      espStartWPS();
      state=WIFI_INITIALIZED;
      apConDlyCtr = 0;
      blStartWPS = false;
      break;
    case WIFI_BULK_TRANSFER:
      blStartBulkTransfer = false;
      // initialize the bulk transfer
      if(initBlkTx() == false) 
      {
        state = WIFI_CONNECTED;
        break;
      }
      runBlkTx();
      stopBlkTx();
      state = WIFI_CONNECTED;
      blGotoSleep = true;
      break;  
    case WIFI_SLEEP:
      // Set sleep pin of esp
      if(blMQTTRunning)
      {
        mqtt_disconnect(&broker);
        blMQTTRunning = false;
      }
      else
      {
        esp8266Sleep(true);
      }
      // check wakeup conditions
      if(blStartBulkTransfer | blSendDataRdy | blStartWPS) state = WIFI_WAKE_UP;
      break;
    case WIFI_WAKE_UP:
      // Set sleep pin of esp
      blGotoSleep = false;
      esp8266Sleep(false);
      state=WIFI_INITIALIZED;
      break;
    case WIFI_TEST_RANGE:
      if(!blMQTTRunning) {initMQTT(); break;}
      if( blMQTTRunning==true )
      {
        UI_SET_LED2(100);
        ret = mqtt_publish(&broker, MQTT_TOPIC, "announce", 0);
        if(ret < 0)
        {
          blMQTTRunning = false;
        }
      }
      break;
    case WIFI_TEST_FILE:
      if(!blMQTTRunning) {initMQTT(); break;}
      if( blMQTTRunning==true )
      {
        fname = "validierung.txt";
        samplerate = 200;
        blStartBulkTransfer = true;
        state = WIFI_BULK_TRANSFER;
      }
      break;
  }
  chMtxUnlock(&mtx);
}

/**
 * @brief      Main WiFi application thread
 *
 */
static THD_FUNCTION(thWiFiApp, arg) {
 (void)arg;
 static systime_t time;
 chRegSetThreadName(DEFS_THD_WIFIAPP_NAME);
 while(true)
 {
  time = chVTGetSystemTime();
  time += MS2ST(THD_INTERVAL);
  fWiFiStateMachine();
  if(time > chVTGetSystemTime()) chThdSleepUntil(time);
 }
}

/**
 * @brief      Inits the wifiapp and starts the thread
 */
static bool wifiAppInit(void)
{
  FIL fp;
  // Read wificonfig file to read access point information
  if(f_open(&fp, WIFI_CFG_PATH, FA_READ) == FR_OK)
  {
    DBG("Config file found\r\n");
    // read first line
    if(f_gets(linebuf, LINE_BUF_SIZE, &fp) == linebuf)
    {
      memset(wifi_ssid, 0, WIFI_SSID_PW_SIZE);
      strncpy(wifi_ssid, linebuf, WIFI_SSID_PW_SIZE);
      rtrim(wifi_ssid, '\r');
      rtrim(wifi_ssid, '\n');
    }
    if(f_gets(linebuf, LINE_BUF_SIZE, &fp) == linebuf)
    {
      memset(wifi_pw, 0, WIFI_SSID_PW_SIZE);
      strncpy(wifi_pw, linebuf, WIFI_SSID_PW_SIZE);
      rtrim(wifi_pw, '\r');
      rtrim(wifi_pw, '\n');
    }
    DBG("Read config file: SSID=\"%s\" PW=\"%s\"\r\n", wifi_ssid, wifi_pw);
    f_close(&fp);
    blDefaultAPAvail = true;
  }
  else
  {
    DBG("Config file NOT found!\r\n");
  }
  return true;
}


/**
 * @brief      Send packet over the socket
 *
 * @param      socket_info  The socket information
 * @param[in]  buf          The buffer
 * @param[in]  count        The count
 *
 * @return     result
 */
static int send_packet(void* socket_info, const void* buf, unsigned int count)
{
  int fd = *((int*)socket_info);
  return send(fd, buf, count, 0);
}

/**
 * @brief      Init the socket
 *
 * @param      broker    The broker
 * @param[in]  hostname  The hostname
 * @param[in]  port      The port
 *
 * @return     status
 */
static int init_socket(mqtt_broker_handle_t* broker, const char* hostname, short port)
{
  int keepalive = 0xffff; // Seconds

  // Create the socket
  if((socket_id = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    return -1;

  // Disable Nagle Algorithm
  // if (setsockopt(socket_id, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(flag)) < 0)
  //   return -2;

  sockaddr_in socket_address;
  // Create the stuff we need to connect
  socket_address.sin_family = AF_INET;
  socket_address.sin_port = htons(port);
  socket_address.sin_addr.s_addr = inet_addr((char*)hostname);

  // Connect the socket
  if((connect(socket_id, (sockaddr*)&socket_address, sizeof(socket_address))) < 0)
    return -1;

  // MQTT stuffs
  mqtt_set_alive(broker, keepalive);
  broker->socket_info = (void*)&socket_id;
  broker->send = send_packet;

  return 0;
}

/**
 * @brief      Close the socket
 *
 * @param      broker  mqtt broker handel
 *
 * @return     close status
 */
static int close_socket(mqtt_broker_handle_t* broker)
{
  int fd = *((int*)broker->socket_info);
  return close(fd);
}

/**
 * @brief      Read a socket packet
 *
 * @param[in]  timeout  The timeout
 *
 * @return     packet length
 */
static int read_packet(int timeout)
{
  (void)timeout;
  int total_bytes = 0, bytes_rcvd, packet_length;
  memset(packet_buffer, 0, sizeof(packet_buffer));

  while(total_bytes < 2) // Reading fixed header
  {
    if((bytes_rcvd = recv(socket_id, (packet_buffer+total_bytes), RCVBUFSIZE, 0)) <= 0)
      return -1;
    total_bytes += bytes_rcvd; // Keep tally of total bytes
  }

  packet_length = packet_buffer[1] + 2; // Remaining length + fixed header length

  while(total_bytes < packet_length) // Reading the packet
  {
    if((bytes_rcvd = recv(socket_id, (packet_buffer+total_bytes), RCVBUFSIZE, 0)) <= 0)
      return -1;
    total_bytes += bytes_rcvd; // Keep tally of total bytes
  }

  return packet_length;
}

/**
 * @brief      Init the MQTT Protocoll stack and open a connection to the broker
 */
static void initMQTT(void)
{
  int packet_length;

  DBG("Starting MQTT Init\r\n");

  mqtt_init(&broker, MQTT_CLIENT_ID);
  // mqtt_init_auth(&broker, "cid", "campeador");
  if(init_socket(&broker, MQTT_SERVER_ADDRESS, MQTT_SERVER_PORT) < 0)
  {
    DBG("Socket create error!\r\n");
    return;
  }

  chThdSleepMilliseconds(1000);
  DBG("Socket connected, connect to broker\r\n");

  // >>>>> CONNECT
  mqtt_connect(&broker);

  // <<<<< CONNACK
  packet_length = read_packet(1);
  if(packet_length < 0)
  {
    DBG("Error(%d) on read packet!\r\n", packet_length);
    return;
  }

  if(MQTTParseMessageType(packet_buffer) != MQTT_MSG_CONNACK)
  {
    DBG("CONNACK expected!\r\n");
    return;
  }

  if(packet_buffer[3] != 0x00)
  {
    DBG("CONNACK failed!\r\n");
    return;
  }

  // >>>>> PUBLISH QoS 0
  DBG("Publish: QoS 0\r\n");
  int ret = mqtt_publish(&broker, MQTT_TOPIC, "announce", 0);
  if(ret < 0)
  {
    DBG("announce publish fail!\r\n");
  }

  blMQTTRunning = true;
  return;
}

/**
 * @brief      Makes things ready for bulk transfer
 */
static bool initBlkTx(void)
{
  // open file
  if(f_open(&fp, fname, FA_READ | FA_OPEN_EXISTING) != FR_OK)
  {
    DBG("Bulk transfer init failed: f_open error\r\n");
    return false;
  }

  DBG("Bulk transfer init success\r\n");
  return true;
}

/**
 * @brief      Runs the bulk transfer, doesnt exit until done
 */
static void runBlkTx(void)
{
  UINT bytesread;
  FRESULT res;
  bool eof;
  uint32_t bytecount = 0;
  systime_t time = 0;
  int ret;
  time = chVTGetSystemTime();

  // send sstart <samplerate>
  chsnprintf(txmsg,TXMSG_SIZE,"sstart %d",samplerate);
  // publishQoS1(&broker, MQTT_TOPIC, txmsg);
  ret = mqtt_publish(&broker, MQTT_TOPIC, txmsg, 0);
  if(ret < 0)
  {
    DBG("Blk Tx error on sstart publish! aborting...\r\n");
    return;
  }
  bytecount += strlen(txmsg);

  // create send string
  eof = false;
  do
  {
    chsnprintf(txmsg,TXMSG_SIZE,"ssample ",samplerate);
    res = f_read(&fp, &txmsg[8], (((BLK_TX_VALS_PER_BLOCK-1)*(BLK_TX_VALUE_LENGHT+1))), &bytesread);
    if(res != FR_OK)
    {
      DBG("Blk Tx f_read error! aborting...\r\n");
      return;
    }
    if(bytesread < ((BLK_TX_VALS_PER_BLOCK-1)*(BLK_TX_VALUE_LENGHT+1)))
    {
      // we are at the end of the file
      eof = true;
    }
    // null terminate the send string
    txmsg[8+bytesread] = 0;
    // publishQoS1(&broker, MQTT_TOPIC, txmsg);
    ret = mqtt_publish(&broker, MQTT_TOPIC, txmsg, 0);
    if(ret < 0)
    {
      DBG("Blk Tx error on ssample publish! aborting...\r\n");
      return;
    }
    bytecount += strlen(txmsg);
    DBG("\rTransmitted: %d bytes", bytecount);
  } while(!eof);

  // send sstop
  chsnprintf(txmsg,TXMSG_SIZE,"sstop");
  // publishQoS1(&broker, MQTT_TOPIC, txmsg);
  ret = mqtt_publish(&broker, MQTT_TOPIC, txmsg, 0);
  if(ret < 0)
  {
    DBG("Blk Tx error on sstop publish! aborting...\r\n");
    return;
  }
  time = chVTGetSystemTime() - time;
  DBG("\rTransmitted: %d bytes in %d seconds\r\n", bytecount, (uint32_t)ST2S(time));
}

/**
 * @brief      Deinitializes the bulk transfer mode
 */
static void stopBlkTx (void)
{
  // close the file
  f_close(&fp);
}

/**
 * @brief      Checks for a QoS 1 puback
 *
 * @param[in]  msg_id  The message identifier
 *
 * @return     true if puback received, false if any error
 */
static bool checkPubackQoS1(uint16_t msg_id)
{
  int packet_length;
  packet_length = read_packet(1);
  if(packet_length < 0)
  {
    DBG("Error(%d) on read packet!\r\n", packet_length);
    return false;
  }

  if(MQTTParseMessageType(packet_buffer) != MQTT_MSG_PUBACK)
  {
    DBG("PUBACK expected!\r\n");
    return false;
  }

  uint16_t msg_id_rcv = mqtt_parse_msg_id(packet_buffer);
  if(msg_id != msg_id_rcv)
  {
    DBG("%d message id was expected, but %d message id was found!\r\n", msg_id, msg_id_rcv);
    return false;
  }
  return true;
}

/**
 * @brief      Transmits a message with QoS1 until it is received
 *
 * @param      br      The broker
 * @param[in]  topic   The topic
 * @param[in]  msg     The message
 */
static void publishQoS1 (mqtt_broker_handle_t* br, const char* topic, const char* msg)
{
  uint8_t dup = 0;
  uint16_t msg_id = 0;
  int ret;
  do
  {
    ret = mqtt_publish_with_qos(br, topic, msg, dup, 1, &msg_id);
    if(ret < 0)
    {
      DBG("Blk Tx error on publishQoS1! aborting...\r\n");
      return;
    }
    dup = 1;
  } while(checkPubackQoS1(msg_id) == false);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief      starts the wifi thread
 */
void wifiStart(void)
{
  espStart();
  chMtxObjectInit(&mtx);
  tp = chThdCreateStatic(waWifi, sizeof(waWifi), 30, thWiFiApp, NULL);
}

/**
 * @brief      Connect to a access point
 *
 * @param[in]  ssid      The ssid
 * @param[in]  password  The password
 */
void wifiAddAP(const char * ssid, const char * password)
{
  if( (strlen(ssid) > WIFI_SSID_PW_SIZE) || (strlen(password) > WIFI_SSID_PW_SIZE) )
  {
    DBG("FATAL wifiConnectAP buf overflow");
    return;
  }
  memcpy(wifi_ssid, ssid, strlen(ssid));
  memcpy(wifi_pw, password, strlen(password));
  blDefaultAPAvail = true;
}

/**
 * @brief      Connects to the default WiFi stored on the sd card
 */
void wifiConnectDefaultAP(void)
{
  if(blDefaultAPAvail == true)
  {
    wifiConnectAP(wifi_ssid, wifi_pw);
  }
}

/**
 * @brief      Publish a single message
 *
 * @param[in]  msg   The message
 */
void wifiPublish(const char* msg)
{
  if(tp == NULL) return;
  if(!lockMtx(&mtx, 10)) return;
  // chMtxLock(&mtx);
  // 
  
  if((blMQTTRunning==true || blGotoSleep) && blSendDataRdy==false && (strlen(msg) <= TXMSG_SIZE))
  {
    memcpy(txmsg, msg, strlen(msg));
    blSendDataRdy = true;
  }
  chMtxUnlock(&mtx);
}

/**
 * @brief      Starts the wps configuration
 */
void wifiStartWPS(void)
{
  if(tp == NULL) return;
  if(!lockMtx(&mtx, 10)) return;

  blStartWPS = true;

  chMtxUnlock(&mtx);
  return;
}

/**
 * @brief      Starts a fast bulk transfer of a samplefile to the mqttt broker
 *
 * @param      f       The filename
 * @param[in]  s  The samplerate
 */
void wifiStartBulkTransfer(char* f, uint16_t s)
{
  if(tp == NULL) return;
  if(!lockMtx(&mtx, 10)) return;

  if( (blMQTTRunning==true || blGotoSleep) && (f!=NULL) )
  {
    fname = f;
    samplerate = s;
    blStartBulkTransfer = true;
  }

  chMtxUnlock(&mtx);
}

/**
 * @brief      Puts the wifi module in sleep mode
 */
void wifiSleep(void)
{
  if(tp == NULL) return;
  if(!lockMtx(&mtx, 10)) return;

  blGotoSleep = true;

  chMtxUnlock(&mtx);
}

/**
 * @brief      Returns the current state machine status
 *
 * @return     state machine state
 */
wifi_state wifiGetState (void)
{
  return state;
}


/** @} */
