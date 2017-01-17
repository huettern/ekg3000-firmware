/*
   EKG3000 - Copyright (C) 2016 FHNW Project 3 Team 2
 */

/**
 * @file       analog.c
 * @brief      Analog module for ECG signal conversion
 * @details    Uses TIM12_CH2 to trigger SDADC3 periodically, filters the data 
 * and stores it on the sdcard
 * 
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       1 December 2016
 * 
 *
 * @addtogroup ANALOG
 * @brief Handles the SDADC Analog to digital conversion
 * @{
 */

#include "analog.h"

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "util.h"
#include "defs.h"

#include "usbcfg.h"

#include "ff.h"
#include "ui.h"
#include "wifiapp.h"

#include <stdlib.h>
#include <string.h>


static BaseSequentialStream * dbgstrm = bssusb;
#if DEFS_ANALOG_DBG == TRUE
  #define DBG(X, ...)    chprintf(dbgstrm, X, ##__VA_ARGS__ )
#else
  #define DBG(X, ...)
#endif

/*===========================================================================*/
/* settings                                                                */
/*===========================================================================*/
/**
 * Define SINGLE_ENDED_CONFIG to read the signal in single ended configuration
 * on SDADC 3 Channel 6, else the differential pair SDADC 3 Channel 8 will
 * be used
 */
#define SINGLE_ENDED_CONFIG TRUE

#ifdef SINGLE_ENDED_CONFIG
  // #define ADC_GRP1_CHANNEL        6 // SDADC 3 Channel 6
  // #define ADC_GPIO_PORT GPIOD
  // #define ADC_GPIO_PIN 8
  #define ADC_GRP1_CHANNEL        8 // SDADC 3 Channel 6
  #define ADC_GPIO_PORT GPIOB
  #define ADC_GPIO_PIN 14
#else
  #define ADC_GRP1_CHANNEL        8 // SDADC 3 Channel 8
#endif

#define SAMPLERATE 1000 // Hz
#define TIMER_BASE_CLOCK 10000
#define DOWNSAMPLING_FACTOR 4
#define ENABLE_AVERAGING TRUE

#define VREF (float)3.3
#define ADC_TO_FLOAT_FACTOR (float)(VREF/(float)0xffff)

#define PULSE_TRSH 0.5
#define WINDOW_SHRINK_FACTOR 0.01

#define SDADC_CR2_JEXTSEL_M(n)  ((n)<<8)
#define SDADC_CR2_JEXTEN_M(n)    ((n)<<13)

#define ADC_GRP1_NUM_CHANNELS   1
#define ADC_GRP1_BUF_DEPTH      64 // buffer size: 64 for 2 32bit fir filters

/**
 * Sample file location
 */
#define FILEPATH "/"


/*===========================================================================*/
/* prototypes                                                                */
/*===========================================================================*/
static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n);
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err);

static void start(void);
static void stop(void);

/*
 * Working area for the analog thread.
 */
static THD_WORKING_AREA(adchreadWorkingArea, DEFS_THD_ANALOG_WA_SIZE);


/*===========================================================================*/
/* private data                                                              */
/*===========================================================================*/
/*
 * SDADC configuration.
 */
static const ADCConfig sdadc_config = 
{
  0, // SDADC CR1 register initialization data.
  { // SDADC CONFxR registers initialization data.
    #ifdef SINGLE_ENDED_CONFIG
      SDADC_CONFR_GAIN_1X | SDADC_CONFR_SE_ZERO_VOLT | SDADC_CONFR_COMMON_VSSSD, //configuration 0
      SDADC_CONFR_GAIN_1X | SDADC_CONFR_SE_ZERO_VOLT | SDADC_CONFR_COMMON_VSSSD, //configuration 1
      SDADC_CONFR_GAIN_1X | SDADC_CONFR_SE_ZERO_VOLT | SDADC_CONFR_COMMON_VSSSD, //configuration 2
    #else
      SDADC_CONFR_GAIN_1X | SDADC_CONFR_COMMON_VSSSD, //configuration 0
      SDADC_CONFR_GAIN_1X | SDADC_CONFR_COMMON_VSSSD, //configuration 1
      SDADC_CONFR_GAIN_1X | SDADC_CONFR_COMMON_VSSSD  //configuration 2
    #endif
  }
};

/*
 * ADC conversion group.
 * Mode:        Continuous, 32 samples of 1 channel, TIMER triggered.
 */
static const ADCConversionGroup adcgrpcfg1 = {
  TRUE, // circular
  ADC_GRP1_NUM_CHANNELS, // num_channels
  adccallback, // end_cb
  adcerrorcallback, // error_cb
  .u.sdadc = {
    /* CR2: select rising edge of TIM12_CH2*/
    SDADC_CR2_JEXTSEL_M(0x02) | SDADC_CR2_JEXTEN_M(0x01),
    /* JCHGR    */
    SDADC_JCHGR_CH(ADC_GRP1_CHANNEL),
    {                       
      #ifdef SINGLE_ENDED_CONFIG
        // SDADC_CONFCHR1_CH6(0),
        // SDADC_CONFCHR1_CH6(0),
        SDADC_CONFCHR2_CH8(0),
        SDADC_CONFCHR2_CH8(0),
      #else
        SDADC_CONFCHR2_CH8(0),
        SDADC_CONFCHR2_CH8(0),
      #endif  
    }
  }
};

static PWMConfig pwmcfg = {
  TIMER_BASE_CLOCK,  /* 10kHz PWM clock frequency.   */
  TIMER_BASE_CLOCK/SAMPLERATE,     /* Initial PWM freq 0.5s       */
  NULL,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },
  0,
  0
};

static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

static float filtered[64];
static float unfiltered[64];

static float umax = 0.0;
static float umin = 3.3;

static volatile adcsample_t* pbuf;

static thread_t *tpAdc;

static uint32_t nSamples = 0;
static uint32_t sampleCtr = 0;

static bool isSampling = false;
static bool blEnableLED = false;

static FIL fp;
static mutex_t mtx;

static char fname[32];


/*===========================================================================*/
/* Module static functions.                                                  */
/*===========================================================================*/
/*
 * analog thread.
 */
static THD_FUNCTION(adcThread, arg) 
{
  (void)arg;
  static uint16_t ctr = 0;
  static uint16_t ctr2 = 0;
  static uint16_t swap = 0;
  static uint16_t of = 0;
  static float mean = 0;
  FRESULT res;
  char str[32];
  UINT size;

  tpAdc = chThdGetSelfX();
  chRegSetThreadName(DEFS_THD_ANALOG_NAME);
  while(true) 
  {
    chEvtWaitAny((eventmask_t)1);

    if(pbuf == samples1) of = 0; else of = ADC_GRP1_BUF_DEPTH/2;

    for(ctr = 0; ctr < ADC_GRP1_BUF_DEPTH/2; ctr++)
    {
      // read data
      #ifdef SINGLE_ENDED_CONFIG
        swap = samples1[ctr + of] ^ 0x8000;
        unfiltered[ctr + of] = ((float)(swap))*ADC_TO_FLOAT_FACTOR;
      #else
        unfiltered[ctr + of] = (float)samples1[ctr + of];
      #endif

      // pulse recognition here
      if(unfiltered[ctr + of] < umin) umin = unfiltered[ctr + of];
      if(unfiltered[ctr + of] > umax) umax = unfiltered[ctr + of];
      if((unfiltered[ctr + of] > (umin+(PULSE_TRSH*(umax-umin)))) && blEnableLED ) UI_SET_LED1(100);
      else if(blEnableLED) UI_SET_LED1(0);
      umin *= 1.0 + WINDOW_SHRINK_FACTOR;
      umax *= 1.0 - WINDOW_SHRINK_FACTOR;

      if(isSampling)
      {
        sampleCtr++;
      }
    }
    if(isSampling)
    {
      #if ENABLE_AVERAGING == TRUE
        for(ctr = 0; ctr < ADC_GRP1_BUF_DEPTH/2; )
        {      
          mean = 0;
          // Build mean value
          for(ctr2 = 0; ctr2 < DOWNSAMPLING_FACTOR; ctr2++)
          {
            mean += unfiltered[ctr + of + ctr2];
          }
          mean /= DOWNSAMPLING_FACTOR;
          // store data
          DBG("\rRead: %d samples", sampleCtr);
          chsnprintf(str,32,"%1.5f ",mean);
          size = strlen(str);
          res = f_write(&fp, str, size, &size);
          if(res != FR_OK) DBG("ADC f_write fail: %d\r\n",res); 
          ctr += DOWNSAMPLING_FACTOR;
        }
      #else
        for(ctr = 0; ctr < ADC_GRP1_BUF_DEPTH/2; )
        {      
          // store data
          DBG("\rRead: %d samples", sampleCtr);
          chsnprintf(str,32,"%1.5f ",unfiltered[ctr + of]);
          size = strlen(str);
          res = f_write(&fp, str, size, &size);
          if(res != FR_OK) DBG("ADC f_write fail: %d\r\n",res); 
          ctr += DOWNSAMPLING_FACTOR;
        }
      #endif
      
      // stop sampling if reached num samples
      if( sampleCtr >= (nSamples-1) )
      {
        stop();
        // init transfer
        wifiStartBulkTransfer(fname,SAMPLERATE/DOWNSAMPLING_FACTOR);
      }
    }
  }
}

/**
 * @brief      Starts the ad sampling
 */
static void start(void)
{
  // open file
  chsnprintf(fname,32,"%s%d.txt",FILEPATH, chVTGetSystemTime());
  if(f_open(&fp, fname, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) 
  {
    DBG("ansaple f_open failed! Returning...\r\n");
    return;
  }
  isSampling = true;
}
/**
 * @brief      stops the ad sampling
 */
static void stop(void)
{
  isSampling = false;
  // close the file
  f_close(&fp);
}

/*===========================================================================*/
/* Module public functions.                                                  */
/*===========================================================================*/
/**
 * @brief      Inits the analog module
 */
void anInit(void)
{
  (void)chThdCreateStatic(adchreadWorkingArea, sizeof(adchreadWorkingArea),
                           NORMALPRIO, adcThread, NULL);

  // Pin configuration
  #ifdef SINGLE_ENDED_CONFIG
    palSetPadMode(ADC_GPIO_PORT, ADC_GPIO_PIN, PAL_MODE_ALTERNATE(0) | PAL_MODE_INPUT_ANALOG );
  #else
    palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(0) | PAL_MODE_INPUT_ANALOG );
    palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(0) | PAL_MODE_INPUT_ANALOG );
  #endif

  chMtxObjectInit(&mtx);

  /*
   * Activates the SDADC1 driver.
   */
  adcStart(&SDADCD3, &sdadc_config);
  adcSTM32Calibrate(&SDADCD3);

  /*
   * Starts an ADC continuous conversion.
   */
  adcStartConversion(&SDADCD3, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);

  pwmStart(&PWMD2, &pwmcfg);
  pwmEnableChannel(&PWMD2, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD2, 5000)); // 50% duty
}

/**
 * @brief      Sample number of samples
 *
 * @param[in]  n     number of samples
 */
void anSampleN(uint32_t n)
{
  if(isSampling) return;
  nSamples = n;
  sampleCtr = 0;
  start();
}

/**
 * @brief      Sample specific time
 *
 * @param[in]  t     Time to sample in [sec]
 */
void anSampleT(uint32_t t)
{
  if(isSampling) return;
  nSamples = t*SAMPLERATE;
  sampleCtr = 0;
  start();
}

/**
 * @brief      Gets the sampling statue
 *
 * @return     Returns true if the analog module is currently sampling
 */
bool anIsSampling(void)
{
  return isSampling;
}

/**
 * @brief      Set the LED status enabled
 *
 * @param[in]  on    true for on, false for off
 */
void anLED(bool on)
{
  blEnableLED = on;
}

/*===========================================================================*/
/* callbacks                                                                 */
/*===========================================================================*/
static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) 
{
  (void)adcp;
  (void)buffer;
  (void)n;
  
  // notify the adc thread
  if(tpAdc==NULL) return;
  chSysLockFromISR();
  pbuf = buffer;
  chEvtSignalI(tpAdc, (eventmask_t)1);
  chSysUnlockFromISR();
}

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) 
{
  (void)adcp;
  (void)err;
}

/** @} */
