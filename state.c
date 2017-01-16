/*
   EKG3000 - Copyright (C) 2016 FHNW Project 3 Team 2
 */

/**
 * @file       state.c
 * @brief      State machine and LED control
 * @details    
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       12 Jan 2017
 *             
 *
 * @addtogroup MAIN
 * @brief Statemachine and LED controll
 * @{
 */

#include "ch.h"
#include "hal.h"

#include "state.h"

#include "wifichannel.h"
#include "ui.h"
#include "defs.h"
#include "wifiapp.h"
#include "sdcard.h"
#include "analog.h"

#include "usbcfg.h"
static BaseSequentialStream * dbgstrm = bssusb;
#if DEFS_STATE_DBG == TRUE
  #define DBG(X, ...)    chprintf(dbgstrm, X, ##__VA_ARGS__ )
#else
  #define DBG(X, ...)
#endif


/*===========================================================================*/
/* settings                                                                */
/*===========================================================================*/
#define AN_SAMPLE_SHORT_TIME 10

/**
 * LED brightness in %
 */
#define LED_BRIGHT 100
#define LED_DARK 0

#define LED_BLINK_PERIOD 200

#define NO_CONNECTION_TIMEOUT 20000 // ms
#define NO_WPS_TIMEOUT 20000 // ms
#define TRANSMIT_TIMEOUT 60000 // ms

/*===========================================================================*/
/* private data                                                              */
/*===========================================================================*/
static uint8_t ledContPhase = 0;
static state_states state = STATE_INITIAL_CONNECT;
static wifi_state wifiState;
static uint32_t timeout_timer = 0; // in ms

/*===========================================================================*/
/* prototypes                                                                */
/*===========================================================================*/
static void ledControl(void);


static void initialConnect(void);
static void wps(void);
static void defaultAp(void);
static void ready(void);
static void record(void);
static void connect(void);
static void transmit(void);
static void fault(void);


/*===========================================================================*/
/* Module static functions.                                                  */
/*===========================================================================*/
/**
 * @brief      Controlls the LED
 * 
 * LED0: Operation and Fault
 * LED1: Controlled by analog module: Pulse
 * LED2: WiFi status led
 * LED3: Analog sampling and WiFi TX led
 */
static void ledControl(void)
{
  if (ledContPhase < (LED_BLINK_PERIOD/2) )
  {
    switch(state)
    {
      case STATE_INITIAL_CONNECT: 
        UI_SET_LED0(LED_BRIGHT); UI_SET_LED2(LED_DARK); UI_SET_LED3(LED_DARK);
        break;
      case STATE_WPS:
        UI_SET_LED0(LED_BRIGHT); UI_SET_LED2(LED_BRIGHT); UI_SET_LED3(LED_DARK);
        break;
      case STATE_DEFAULT_AP:
        UI_SET_LED0(LED_BRIGHT); UI_SET_LED2(LED_BRIGHT); UI_SET_LED3(LED_DARK);
        break;
      case STATE_READY:
        UI_SET_LED0(LED_BRIGHT); UI_SET_LED2(LED_BRIGHT); UI_SET_LED3(LED_DARK);
        break;
      case STATE_RECORD:
        UI_SET_LED0(LED_BRIGHT); UI_SET_LED2(LED_BRIGHT); UI_SET_LED3(LED_BRIGHT);
        break;
      case STATE_CONNECT:
        UI_SET_LED0(LED_BRIGHT); UI_SET_LED2(LED_BRIGHT); UI_SET_LED3(LED_BRIGHT);
        break;
      case STATE_TRANSMIT:
        UI_SET_LED0(LED_BRIGHT); UI_SET_LED2(LED_BRIGHT); UI_SET_LED3(LED_BRIGHT);
        break;
      case STATE_FAULT:
        UI_SET_LED0(LED_BRIGHT); UI_SET_LED2(LED_DARK); UI_SET_LED3(LED_DARK);
        break;
    }
  }
  else
  {
    switch(state)
    {
      case STATE_INITIAL_CONNECT: 
        UI_SET_LED0(LED_BRIGHT); UI_SET_LED2(LED_DARK); UI_SET_LED3(LED_DARK);
        break;
      case STATE_WPS:
        UI_SET_LED0(LED_BRIGHT); UI_SET_LED2(LED_DARK); UI_SET_LED3(LED_DARK);
        break;
      case STATE_DEFAULT_AP:
        UI_SET_LED0(LED_BRIGHT); UI_SET_LED2(LED_DARK); UI_SET_LED3(LED_DARK);
        break;
      case STATE_READY:
        UI_SET_LED0(LED_BRIGHT); UI_SET_LED2(LED_BRIGHT); UI_SET_LED3(LED_DARK);
        break;
      case STATE_RECORD:
        UI_SET_LED0(LED_BRIGHT); UI_SET_LED2(LED_BRIGHT); UI_SET_LED3(LED_BRIGHT);
        break;
      case STATE_CONNECT:
        UI_SET_LED0(LED_BRIGHT); UI_SET_LED2(LED_BRIGHT); UI_SET_LED3(LED_DARK);
        break;
      case STATE_TRANSMIT:
        UI_SET_LED0(LED_BRIGHT); UI_SET_LED2(LED_BRIGHT); UI_SET_LED3(LED_DARK);
        break;
      case STATE_FAULT:
        UI_SET_LED0(LED_DARK); UI_SET_LED2(LED_DARK); UI_SET_LED3(LED_DARK);
        break;
    }
  }

  ledContPhase += STATE_MACHINE_INTERVAL_MS;
  ledContPhase %= LED_BLINK_PERIOD;
}

/**
 * @brief      Tries to connect to the WiFi last used
 */
static void initialConnect(void)
{
  // check exit condition: connected
  if(wifiState == WIFI_SLEEP || wifiState == WIFI_CONNECTED)
  {
    state = STATE_READY;
  }
  if(gets2pushed())
  {
    wifiStartWPS();
    state = STATE_WPS;
  }
  if(gets2longPushed())
  {
    wifiConnectDefaultAP();
    state = STATE_DEFAULT_AP;
  }
  if(timeout_timer >= NO_CONNECTION_TIMEOUT)
  {
    state = STATE_FAULT;
  }
}
/**
 * @brief      starts the WPS routine and waits for success
 */
static void wps(void)
{
  // check exit condition: connected
  if(wifiState == WIFI_SLEEP || wifiState == WIFI_CONNECTED)
  {
    state = STATE_READY;
  }
  if(timeout_timer >= NO_WPS_TIMEOUT)
  {
    state = STATE_FAULT;
  }
}
/**
 * @brief      connects to the default ap and waits for success
 */
static void defaultAp(void)
{
  // check exit condition: connected
  if(wifiState == WIFI_SLEEP || wifiState == WIFI_CONNECTED)
  {
    state = STATE_READY;
  }
  if(timeout_timer >= NO_WPS_TIMEOUT)
  {
    state = STATE_FAULT;
  }
}
/**
 * @brief      device is ready for a measurement
 */
static void ready(void)
{
  if(wifiState == WIFI_SLEEP) // this if should be obsolete, just for safety...
  {
    // if(gets0pushed()) 
    // {
    //   anSampleT(60);
    //   state = STATE_RECORD;
    // }
    if(gets1pushed()) 
    {
      anSampleT(AN_SAMPLE_SHORT_TIME);
      state = STATE_RECORD;
    }
  }
}
/**
 * @brief      Recording a measurement
 */
static void record(void)
{
  // if sampling is done, change state
  if(anIsSampling() == false)
  {
    state = STATE_CONNECT;
  } 
}
/**
 * @brief      Connecting to the server for transmitting
 */
static void connect(void)
{
  if(wifiState == WIFI_BULK_TRANSFER)
  {
    state = STATE_TRANSMIT;
  }
  if(timeout_timer >= NO_CONNECTION_TIMEOUT)
  {
    state = STATE_FAULT;
  }
}
/**
 * @brief      transmitting data
 */
static void transmit(void)
{
  if(wifiState != WIFI_BULK_TRANSFER)
  {
    state = STATE_READY;
  }
  if(timeout_timer >= TRANSMIT_TIMEOUT)
  {
    state = STATE_FAULT;
  }
}
/**
 * @brief      faulty state
 */
static void fault(void)
{
  anLED(false);
  UI_SET_LED1(0);
  if(gets2pushed())
  {
    NVIC_SystemReset();
  }
}

/*===========================================================================*/
/* callbacks                                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module public functions.                                                  */
/*===========================================================================*/
/**
 * @brief      Inits the state machine
 */
void stateInit(void)
{
  UI_SET_LED0(LED_BRIGHT); // system ready
  timeout_timer = 0;
}

/**
 * @brief      Runs the state machine, call periodically
 */
void stateRun(void)
{
  state_states oldState = state;
  wifiState = wifiGetState();
  timeout_timer += STATE_MACHINE_INTERVAL_MS;

  ledControl();

  switch(state)
  {
    case STATE_INITIAL_CONNECT:
      DBG("STATE: In state STATE_INITIAL_CONNECT\r\n");
      initialConnect();
      break;
    case STATE_WPS:
      DBG("STATE: In state STATE_WPS\r\n");
      wps();
      break;
    case STATE_DEFAULT_AP:
      DBG("STATE: In state STATE_DEFAULT_AP\r\n");
      defaultAp();
      break;
    case STATE_READY:
      DBG("STATE: In state STATE_READY\r\n");
      ready();
      break;
    case STATE_RECORD:
      DBG("STATE: In state STATE_RECORD\r\n");
      record();
      break;
    case STATE_CONNECT:
      DBG("STATE: In state STATE_CONNECT\r\n");
      connect();
      break; 
    case STATE_TRANSMIT:
      DBG("STATE: In state STATE_TRANSMIT\r\n");
      transmit();
      break;
    case STATE_FAULT:
      DBG("STATE: In state STATE_FAULT\r\n");
      fault();
      break;
  }

  if(oldState != state)
  {
    timeout_timer = 0;
  }

}

/** @} */
