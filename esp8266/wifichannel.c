/*
   EKG3000 - Copyright (C) 2016 FHNW Project 3 Team 2
 */

/**
 * @file       wifichannel.c
 * @brief      handels the differenc TCP/UDP channels of the esp module
 * @details    Source: https://github.com/vpcola/MikroChibiOS
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       1 December 2016
 * 
 *
 * @addtogroup WIFICHANNEL
 * @brief Wifi channel management
 * @{
 */

#include "wifichannel.h"
#include "ch.h"
#include "hal.h"
#include "esp8266.h"
#include "chprintf.h"
#include "defs.h"

#include <chqueues.h>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>


#include "usbcfg.h"


/*===========================================================================*/
/* settings                                                                */
/*===========================================================================*/
#define WA_SIZE DEFS_THD_WIFICHANNEL_WA_SIZE

/**
 * Queue buffer size is MTU size
 */
#define QUEUEBUF_SIZ0 1500
#define QUEUEBUF_SIZ1 100
#define QUEUEBUF_SIZ2 100
#define QUEUEBUF_SIZ3 100
#define QUEUEBUF_SIZ4 100

#define RCV_THREA_INTERVAL_MS 20

#if DEFS_WIFICHANNEL_DBG == TRUE
  static BaseSequentialStream * dbgstrm = bssusb;
  #define DBG(X, ...)    chprintf(dbgstrm, X, ##__VA_ARGS__ )
#else
  #define DBG(X, ...)
#endif

/*===========================================================================*/
/* private data                                                              */
/*===========================================================================*/
// A fixed static array of input queues
// Here we define the static buffers for
// our input queues
static uint8_t queue_buff0[QUEUEBUF_SIZ0]  __attribute__ ((section(".bufram")));
static uint8_t queue_buff1[QUEUEBUF_SIZ1]  __attribute__ ((section(".bufram")));
static uint8_t queue_buff2[QUEUEBUF_SIZ2]  __attribute__ ((section(".bufram")));
static uint8_t queue_buff3[QUEUEBUF_SIZ3]  __attribute__ ((section(".bufram")));
static uint8_t queue_buff4[QUEUEBUF_SIZ4]  __attribute__ ((section(".bufram")));

static void notify(io_queue_t *qp) {
  (void)qp;
}

// Our array of input queues, actually no need to do this since Init
// is called later.
static INPUTQUEUE_DECL(iq0, queue_buff0, QUEUEBUF_SIZ0, notify, NULL);
static INPUTQUEUE_DECL(iq1, queue_buff1, QUEUEBUF_SIZ1, notify, NULL);
static INPUTQUEUE_DECL(iq2, queue_buff2, QUEUEBUF_SIZ2, notify, NULL);
static INPUTQUEUE_DECL(iq3, queue_buff3, QUEUEBUF_SIZ3, notify, NULL);
static INPUTQUEUE_DECL(iq4, queue_buff4, QUEUEBUF_SIZ4, notify, NULL);

esp_channel_t _esp_channels[MAX_CONNECTIONS] = {
      { 0, ESP_TCP, CHANNEL_UNUSED, false, "", 0, "127.0.0.1", 0, false, &iq0 , 0},
      // { 1, ESP_TCP, CHANNEL_UNUSED, false, "", 0, "127.0.0.1", 0, false, &iq1 , 0},
      // { 2, ESP_TCP, CHANNEL_UNUSED, false, "", 0, "127.0.0.1", 0, false, &iq2 , 0},
      // { 3, ESP_TCP, CHANNEL_UNUSED, false, "", 0, "127.0.0.1", 0, false, &iq3 , 0},
      // { 4, ESP_TCP, CHANNEL_UNUSED, false, "", 0, "127.0.0.1", 0, false, &iq4 , 0},
};

// uart write mutex
static MUTEX_DECL(usartmtx);

static THD_WORKING_AREA(channelListenerThreadWA, WA_SIZE);

/*===========================================================================*/
/* Module static functions.                                                  */
/*===========================================================================*/
/**
 * @brief      Reset a channel to defaults
 *
 * @param      ch    channel pointer
 */
static void resetChannel(esp_channel_t * ch)
{
    if (ch)
    {
        ch->status = CHANNEL_UNUSED;
        ch->port = 0;
        ch->localport = 0;
        strncpy(ch->ipaddress, "", IPADDR_MAX_SIZ);
        strncpy(ch->localaddress, "127.0.0.1", IPADDR_MAX_SIZ);
        ch->isservergenerated = false;
        ch->ispassive = false;
        ch->type = ESP_TCP;
        ch->usecount = 0;
    }
}

/**
 * @brief      Called on a connection status rx from the esp
 *
 * @param      constatus  The constatus
 */
static void onConnectionStatus(espConStatus_t * constatus)
  {
  if(constatus)
  {
    esp_channel_t * ch = getChannel(constatus->id);

    DBG(     ">> Id[%d] Type[%d] IP [%s], port[%d], isserver[%d]\r\n",
    constatus->id, constatus->type, constatus->srcaddress,
    constatus->port, constatus->clisrv);
    if (ch)
    {
      // check if this is a newly connected channel
      if (ch->status != CHANNEL_CONNECTED)
      {
        // new connection, populate details here
        ch->status = CHANNEL_CONNECTED;
        ch->type = constatus->type;
        ch->isservergenerated = (constatus->clisrv == 1);
        strcpy(ch->ipaddress, constatus->srcaddress);
        ch->port = constatus->port;
        ch->usecount = 0; // clients first task is to increment this.
        // TODO: We need a way to signal waiting threads that
        // a new connection is available i.e. used by accept()
        // call. For ChibiOS, a semaphore might be appropriate
      }
    }
  }
}

/**
 * @brief      Called on line status update from the esp
 *
 * @param      ipstatus  The ipstatus
 */
static void onLineStatus(espIPStatus_t * ipstatus)
{

  for (int i = 0; i < MAX_CONNECTIONS; i++ )
  {
      esp_channel_t * ch = getChannel(i);
      if (ch)
      {
          // If we have a connection that has changed
          // status
          if ((ipstatus->status[i] == ESP_WIFI_CONN_DISCONNECTED)
              && (ch->status == CHANNEL_CONNECTED))
              resetChannel(ch);

          switch(ipstatus->status[i])
          {
              case ESP_WIFI_CONN_GOTIP:
              case ESP_WIFI_CONN_CONNECTED:
                ch->status = CHANNEL_CONNECTED;
                break;
              case ESP_WIFI_CONN_DISCONNECTED:
              default:
                ch->status = CHANNEL_DISCONNECTED;
          }
      }
  }
}

/**
 * @brief      Channel thread to handle received messages
 *
 */
static THD_FUNCTION(channelListenerThread, arg)
{
  // continuously reads the tx circular buffers for
  // each connection and send data to wifi chip
  (void)arg;
  int numread, numbytes, numwritten;
  uint32_t retval;
  int chanid, c; //, pollcount = 0;
  esp_channel_t * channel;
  msg_t retmsg;
  // SerialDriver * sdp = getSerialDriver();

  chRegSetThreadName(DEFS_THD_WIFICHANNEL_NAME);

  while(1)
  {
    // Let other threads run ...
    chThdSleepMilliseconds(RCV_THREA_INTERVAL_MS);

    // Check if there's a message pending on the usart
    // DBG("<<Waiting for data...\r\n");
    if (!chMtxTryLock(&usartmtx))
    {
      chThdSleepMicroseconds(10);
      continue;
    }

    // DBG("<< Got lock...\r\n");
    if (esp8266HasData())
    {
      DBG("<< Got data ...\r\n!");
      // Read the data and determine to which connection
      // it needs to be sent to
      retval = esp8266ReadRespHeader(&chanid, &numbytes, 2000);
      if ((numbytes > 0) && (retval == ESP_RET_IPD))
      {
        channel = getChannel(chanid);
        DBG("<< %d Data on channel %d\r\n", numbytes, chanid);
    
        // Read and push the data to the input queue of the
        // designated connection.
        if (channel)
        {
          DBG("<< Reading %d data ...\r\n", numbytes);
          numread = 0; numwritten = 0;
          while(numread < numbytes)
          {
            c = esp8266Get(0);
            if (c >= 0) 
            {
              chSysLock();
              retmsg = chIQPutI(channel->iqueue, c);
              chSysUnlock();
              if (retmsg == Q_OK) numwritten++;
              numread ++;
            }
          }
          DBG("<< Wrote %d data to queue\r\n", numwritten);
        }
      }

      if ((retval == ESP_RET_UNLINK) || (retval == ESP_RET_LINKED))
      {
        // If we receive an "Unlink" after reading,
        // we need to issue a CIPSTATUS in order to let us
        // know what channel it belongs...
        //esp8266CmdCallback("AT+CIPSTATUS", "OK\r\n", onLineStatus);
        // esp8266GetIpStatus(onLineStatus,onConnectionStatus);
      }
      if (retval == ESP_RET_CONNECT)
      {
        channel = getChannel(chanid);
        channel->status = CHANNEL_CONNECTED;
      }
      if (retval == ESP_RET_CLOSED)
      {
        channel = getChannel(chanid);
        channel->status = CHANNEL_DISCONNECTED;
      }
    } // has data
    chMtxUnlock(&usartmtx);
  }
}


/*===========================================================================*/
/* Module public functions.                                                  */
/*===========================================================================*/
/**
 * @brief      Return the esp txrx channel handle
 *
 * @param[in]  d channel number
 *
 * @return     The channel.
 */
esp_channel_t * getChannel(int d)
{
    if ((d >= 0 ) && ( d < MAX_CONNECTIONS))
        return &_esp_channels[d];

    return NULL;
}

/**
 * @brief      Channel open returns an unused, empty channel
 *
 * @param[in]  conntype  The conntype
 *
 * @return     number of empty channel
 */
int channelOpen(int conntype)
{
    for (int i = 0; i < MAX_CONNECTIONS; i++)
    {
        DBG( "<< Channel %d is %d\r\n",
                 _esp_channels[i].id,
                  _esp_channels[i].status );

        if ((_esp_channels[i].status == CHANNEL_UNUSED) ||
            (_esp_channels[i].status == CHANNEL_DISCONNECTED))
        {
            _esp_channels[i].status = CHANNEL_DISCONNECTED;
            _esp_channels[i].type = conntype;
            DBG( "<< Opening channel[%d] with type %d\r\n",
                     i, conntype);

            return i;
        }
    }
    return -1;
}

/**
 * @brief      Connect to a server on a channel. Before this, open the channel
 * using channelOpen 
 *
 * @param[in]  channel    The channel
 * @param[in]  ipaddress  The ipaddress
 * @param[in]  port       The port
 *
 * @return     status
 */
int channelConnect(int channel, const char * ipaddress, uint16_t port)
{
    int status = -1, retval;
    esp_channel_t * ch = getChannel(channel);

    if(!ch) return -1; // channel does not exist

    if (ch->status == CHANNEL_CONNECTED) 
    {
        return -2; // Channel already connected
    }

    // A connection request needs to lock the usart
    DBG("<< Acquiring lock ...\r\n");
    chMtxLock(&usartmtx);

    DBG( "<< Opening channel[%d] [%s:%d] with %d\r\n",
             channel, ipaddress, port, ch->type);

    retval = esp8266Connect(channel, ipaddress, port, ch->type);
    // chprintf((BaseSequentialStream *)dbgstrm, "<< Connect returned %d!\r\n", retval);
    if ((retval == ESP_RET_OK) || (retval == ESP_RET_LINKED))
    {
      DBG("<< Channel connected!\r\n");
        // Once connected, set the status of the channel
        // array.
        ch->status = CHANNEL_CONNECTED;
        ch->port = port;
        strncpy(ch->ipaddress, ipaddress, IPADDR_MAX_SIZ); 
        ch->usecount++;
        ch->ispassive = false;
        ch->isservergenerated = false;
        // reset the queue
        chSysLock();
        chIQResetI(ch->iqueue);
        chSysUnlock();

        status = 0; // no error, connected
    }

    // Unlock the usart here
    chMtxUnlock(&usartmtx);

    // return with status
    return status;
}

/**
 * @brief      check if selected channel is connected
 *
 * @param[in]  channel  The channel
 *
 * @return     true if connected
 */
bool channelIsConnected(int channel)
{
  esp_channel_t * ch = getChannel(channel);
  if (ch) return (ch->status == CHANNEL_CONNECTED);

  return false;
}

/**
 * @brief      Start an openned channel as server
 *
 * @param[in]  channel  The channel
 * @param[in]  type     The type
 * @param[in]  port     The port
 *
 * @return     { description_of_the_return_value }
 */
int channelServer(int channel, int type, uint16_t port)
{
    // to create a ESP_tcp server, we don't need a
    // channel id and type (since for now it only 
    // supports ESP_TCP server)
    esp_channel_t * ch = getChannel(channel);

    if(!ch) return -1;

    // this must be a passive channel
    if (!ch->ispassive) return -1;

    // lock the usart
    chMtxLock(&usartmtx);

    if (ch && (esp8266Server(channel, type, port) == ESP_RET_OK))
    {
        // passive socket doesn't have a meaning for
        // the esp8266, we only use this to pass parameters
        // and to simulate a bind-listen-accept socket 
        // like emulation so after we create a server 
        // connection on the esp8266, we free up this channel.
        resetChannel(ch);
    }

    chMtxUnlock(&usartmtx);

    return 0;
}

/**
 * @brief      Close a given channel
 *
 * @param[in]  channel  The channel
 *
 * @return     { description_of_the_return_value }
 */
int channelClose(int channel)
{
    int retval = -1;

    esp_channel_t * ch = getChannel(channel);

    if (!ch) return -1;

    // lock the usart
    chMtxLock(&usartmtx);

    if (esp8266Disconnect(channel))
    {
        resetChannel(ch);
        retval = 0;
    }
    
    chMtxUnlock(&usartmtx);
    
    return retval;
}

/**
 * @brief      Send a line through the channel
 *
 * @param[in]  channel  The channel
 * @param[in]  msg      The message
 *
 * @return     { description_of_the_return_value }
 */
bool channelSendLine(int channel, const char * msg)
{
  bool result = false;

  if (channelIsConnected(channel))
  {
    chMtxLock(&usartmtx);
    result = esp8266SendLine(channel, msg);
    chMtxUnlock(&usartmtx);
    return result;
  }

  return result;
}

/**
 * @brief      Send a message through the channel
 *
 * @param[in]  channel  The channel
 * @param[in]  msg      The message
 * @param[in]  msglen   The msglen
 *
 * @return     { description_of_the_return_value }
 */
int channelSend(int channel, const void * msg, int msglen)
{
    int numsend = -1;

    if (channelIsConnected(channel))
    {
      // Lock the usart ...
      chMtxLock(&usartmtx);

      DBG(">>Sending Header ...\r\n");
      // Send the message with a blocking call...
      if ( esp8266SendHeader(channel, msglen))
      {
        esp_channel_t * ch = getChannel(channel);
        DBG(">>Now Sending data\r\n");
        // For UDP, response is received sometimes right away
        // before we even get the "SEND OK" line
        if(esp8266Send(msg, msglen, (ch->type == ESP_TCP)) == true) numsend = msglen;
      }

      // Unlock the usart ...
      chMtxUnlock(&usartmtx);
    }
    return numsend;
}

/**
 * @brief      Open a new connection on an open channel and send a message
 *
 * @param[in]  channel    The channel
 * @param[in]  msg        The message
 * @param[in]  msglen     The msglen
 * @param[in]  ipaddress  The ipaddress
 * @param[in]  port       The port
 *
 * @return     number of characters sent
 */
int channelSendTo(int channel, const char * msg, int msglen, const char * ipaddress, uint16_t port)
{
  esp_channel_t * ch = getChannel(channel);
  if(ch)
  {
    if (channelConnect(channel, ipaddress, port) < 0)
      return -1;
    return channelSend(channel, msg, msglen);
  }

  return -1;
}

/**
 * @brief      Read from a channel
 *        Channel read reads from an input queue if data
 *        is available, otherwise it blocks until all msglen
 *        data is read.
 *
 * @param[in]  chanid  The chanid
 * @param      buff    The buffer
 * @param[in]  msglen  The msglen
 *
 * @return     number of characters read
 */
int channelRead(int chanid, char * buff, int msglen)
{
    
    int numread = 0;
    int data;
    esp_channel_t * channel = getChannel(chanid);

    DBG(">>Reading data from queue\r\n");

    if (!channel)
    {
      DBG(">>Invalid channel\r\n");
      return 0;
    }

    // If disconnected and no data currently on queue
    chSysLock();
    if (!channelIsConnected(chanid) && chIQIsEmptyI(channel->iqueue))
      return -1;
    chSysUnlock();

    // Quirk ... we need to wait indefinitely
    // on the first byte read ..
    buff[numread] = chIQGet(channel->iqueue);
    numread++;

    // Read from the input queue when there's data
    do{
      data = chIQGetTimeout(channel->iqueue,1000);
      if (data >= 0)
      {
        buff[numread] = data;
        numread++;
      }
    }while((data >= 0) && (numread < msglen));

    DBG(">>Read %d data from queue\r\n", numread);
    return numread;
}

/**
 * @brief      Read a line from the channel
 *
 * @param[in]  chanid  The chanid
 * @param      buff    The buffer
 * @param[in]  buflen  The buflen
 *
 * @return     number of characters read
 */
int channelReadLine(int chanid, char * buff, int buflen)
{
    int data, numread = 0;
    esp_channel_t * channel = getChannel(chanid);
    if (channel)
    {
        // Read from the input queue when there's data
        do{
            data = chIQGet(channel->iqueue);
            if (data <= 0) break;
            buff[numread] = (char) data;
            numread++;

            // If newline is reached, 
            // return with numread.
            if ((numread >= 2) &&
                    (buff[numread-2] == '\r') &&
                    (buff[numread-1] == '\n'))
            {
                numread -= 2;
                buff[numread] = 0;
                break;
            }
        }while(numread < buflen);
    } else 
        return -1;

    return numread;
}

/**
 * @brief      Get single character from input
 *
 * @param[in]  chanid  The chanid
 *
 * @return     returns channel
 */
int channelGet(int chanid)
{
  esp_channel_t * channel = getChannel(chanid);

  chSysLock();
  if (!channelIsConnected(chanid) && chIQIsEmptyI(channel->iqueue))
  {
    chSysUnlock();
    return -1;
  }

  if (channel)
    return chIQGet(channel->iqueue);

  return -1;
}

/**
 * @brief      Init the wifi hardware module, start the receive thread
 *
 * @return     ESP_RET status
 */
int wifiInit()
{
  int ret = espInit();

  if(ret == ESP_RET_OK)
  {
    // Start the channel read thread
    chThdCreateStatic(channelListenerThreadWA, 
      sizeof(channelListenerThreadWA),
      NORMALPRIO, 
      channelListenerThread, 
      NULL);
  }

  return ret;
}

/**
 * @brief      Checks if the WiFi module has an IP address
 *
 * @return     true if IP is available
 */
bool wifiHasIP()
{
  return espHasIP();
}

/**
 * @brief      Connect to a WiFi Hotspot
 *
 * @param[in]  ssid      The ssid
 * @param[in]  password  The password
 *
 * @return     { description_of_the_return_value }
 */
int wifiConnectAP(const char * ssid, const char * password)
{
    int conresult = espConnectAP(ssid, password);
    if (conresult != ESP_RET_OK)
    {
        // handle an error here
        return -1;
    }

    return 0;
}

/**
 * @brief      Print channel status to output stream
 *
 * @param      chp   The output stream
 */
void channelStatus(BaseSequentialStream *chp)
{
  chprintf(chp, "id   sta   ip   port   cnt\r\n");
  for(int i = 0; i < MAX_CONNECTIONS; i++)
  {
    chprintf(chp, "%d   %d   %s   %d   %d\r\n",
      _esp_channels[i].id,
      _esp_channels[i].status,
      _esp_channels[i].ipaddress,
      _esp_channels[i].port,
      _esp_channels[i].usecount);
  }
}

/**
 * @brief      Sets the cipmux aftera  sleep to open tcp client channels
 */
bool channelSetCipmux (void)
{
  bool ret;
  // Lock the usart ...
  chMtxLock(&usartmtx);
  ret = esp8266Cmd("AT+CIPMUX=1", "OK", 500);
  chMtxUnlock(&usartmtx);
  return ret;
}


/** @} */