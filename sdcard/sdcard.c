/*
   EKG3000 - Copyright (C) 2016 FHNW Project 3 Team 2
 */

/**
 * @file       sdcard.c
 * @brief      Provides SD Card access and init functinos
 * @details    
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       1 December 2016
 * 
 *
 * @addtogroup SDCARD
 * @brief SD Card 
 * @{
 */

#include "sdcard.h"

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "util.h"

#include "usbcfg.h"

#include "ff.h"
#include "ui.h"

#include <stdlib.h>
#include <string.h>


static BaseSequentialStream * dbgstrm = bssusb;
#if DEFS_SDC_DBG == TRUE
  #define DBG(X, ...)    chprintf(dbgstrm, X, ##__VA_ARGS__ )
#else
  #define DBG(X, ...)
#endif

/*===========================================================================*/
/* settings                                                                */
/*===========================================================================*/
// CS pin settings
#define CS_PORT GPIOA
#define CS_PIN 4

// SD Card insert pin
#define SD_INS_PORT GPIOB
#define SD_INS_PIN 1

/*===========================================================================*/
/* private data                                                              */
/*===========================================================================*/
/* MMC Driver */
MMCDriver MMCD1;

/* Maximum speed SPI configuration (??MHz, CPHA=0, CPOL=0, MSb first).*/
static SPIConfig hs_spicfg = {NULL, GPIOA, CS_PIN, 
                0, // CR1
                0}; // CR2

/* Low speed SPI configuration (??Hz, CPHA=0, CPOL=0, MSb first).*/
static SPIConfig ls_spicfg = {NULL, GPIOA, CS_PIN,
                              (SPI_CR1_BR_2 | SPI_CR1_BR_1), // CR1
                              0}; // CR2

/* MMC/SD over SPI driver configuration.*/
static MMCConfig mmccfg = {&SPID1, &ls_spicfg, &hs_spicfg};

// FS Object
static FATFS SDC_FS;

/* FS mounted and ready.*/
static bool fs_ready = FALSE;

/* Generic large buffer.*/
static uint8_t fbuff[1024] __attribute__ ((section(".bufram")));

/*===========================================================================*/
/* callbacks                                                               */
/*===========================================================================*/


/*===========================================================================*/
/* Module static functions.                                                  */
/*===========================================================================*/
/**
 * @brief      Print file system tree to stream
 *
 * @param      chp   The stream
 * @param      path  The path
 *
 * @return     FRESULT of read operations
 */
static FRESULT scan_files(BaseSequentialStream *chp, char *path) {
  FRESULT res;
  FILINFO fno;
  DIR dir;
  int i;
  char *fn;

#if _USE_LFN
  fno.lfname = 0;
  fno.lfsize = 0;
#endif
  res = f_opendir(&dir, path);
  if (res == FR_OK) {
    i = strlen(path);
    for (;;) {
      res = f_readdir(&dir, &fno);
      if (res != FR_OK || fno.fname[0] == 0)
        break;
      if (fno.fname[0] == '.')
        continue;
      fn = fno.fname;
      if (fno.fattrib & AM_DIR) {
        path[i++] = '/';
        strcpy(&path[i], fn);
        res = scan_files(chp, path);
        if (res != FR_OK)
          break;
        path[--i] = 0;
      }
      else {
        chprintf(chp, "%s/%s\r\n", path, fn);
      }
    }
  }
  return res;
}

/*===========================================================================*/
/* Module public functions.                                                  */
/*===========================================================================*/
/**
 * @brief      Inits the SD Card driver and file system
 */
void sdcInit(void)
{
  DBG("Starting SD Card Init\r\n");

  // PIN configurations
  palSetPadMode(GPIOA, 5, PAL_MODE_ALTERNATE(5) | PAL_MODE_INPUT_PULLUP); // SCK
  palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(5) | PAL_MODE_INPUT_PULLUP); // MISO
  palSetPadMode(GPIOB, 0, PAL_MODE_ALTERNATE(5)); // MOSI
  palSetPadMode(CS_PORT, CS_PIN, PAL_MODE_ALTERNATE(5) | PAL_MODE_INPUT_PULLUP); // CS
  palSetPadMode(SD_INS_PORT, SD_INS_PIN, PAL_MODE_INPUT);
  

  mmcObjectInit(&MMCD1);
  mmcStart(&MMCD1, &mmccfg);

  // try 10 times to connect SD Card
  sdcMount();
}

/**
 * @brief      Prints the filesystem tree to the Stream
 *
 * @param      chp   The output stream
 */
void sdcTree(BaseSequentialStream *chp) {  
  FRESULT err;
  FATFS *fsp; 
  uint32_t clusters;


  if (!fs_ready) {
    chprintf(chp, "File System not mounted\r\n");
    return;
  }

  err = f_getfree("/", &clusters, &fsp);
  if (err != FR_OK) {
    chprintf(chp, "FS: f_getfree() failed\r\n");
    return;
  }
  chprintf(chp,
           "FS: %lu free clusters, %lu sectors per cluster, %lu bytes free\r\n",
           clusters, (uint32_t)SDC_FS.csize,
           clusters * (uint32_t)SDC_FS.csize * (uint32_t)MMCSD_BLOCK_SIZE);
  fbuff[0] = 0;
  scan_files(chp, (char *)fbuff);
}

/**
 * @brief      Umounts the filesystem
 */
void sdcUnmount (void)
{
  f_mount(NULL, "/", 0);
  mmcDisconnect(&MMCD1);
  // mmcStop(&MMCD1);
  fs_ready = FALSE;
}


/**
 * @brief      Mounts the filesystem
 */
void sdcMount (void)
{
  FRESULT err;
  uint8_t ctr;
  
  if(fs_ready) return;

  for(ctr = 0; ctr < 10; ctr++)
  {
    if (mmcConnect(&MMCD1) == HAL_SUCCESS) 
    {
      break;
    }
  }

  if(ctr >= 10)
  {
    DBG("SD CARD FAILED\r\n");
    return;
  }
  
  else 
  {
    DBG("SD CARD SUCCEEDED\r\n");
    err = f_mount(&SDC_FS, "/", 1);
    if (err != FR_OK) {
      mmcDisconnect(&MMCD1);
      return;
    }
    fs_ready = TRUE;
  }
}

/**
 * @brief      Returns the mount status of the fs
 *
 * @return     true if mounted
 */
bool sdcIsMounted (void) 
{
  return fs_ready;
}

#if _USE_MKFS
/**
 * @brief      Unmounts the fs, creates a filesystem and mounts it again
 *
 * @return     status of the formatting
 */
FRESULT sdcMkfs (void)
{
  static BYTE work[_MAX_SS]; /* Work area (larger is better for processing time) */
  FRESULT res;

  if(fs_ready)
  {
    sdcUnmount();
  }

  // res = f_mkfs("", FM_FAT32, 0, work, sizeof(work));
  res = f_mkfs("", 0, 4096);

  if(res != FR_OK)
  {
    DBG("FORMATTING FAILED\r\n");
    return res;
  }

  DBG("FORMATTING SUCCESS\r\n");
  sdcMount();

  return res;
}
#endif // #if _USE_MKFS

/**
 * @brief      Performs a mount-open-write-close-unmount benchmark
 *
 * @param[in]  it    How many iteration to do
 */
void sdcBenchmark (BaseSequentialStream *chp, uint8_t it)
{
  systime_t time = 0;
  bool error = false;
  FIL fp;
  UINT size;
  bool wasMounted = sdcIsMounted();

  if(sdcIsMounted()) sdcUnmount();

  // mmcStart(&MMCD1, &mmccfg);
  if (mmcConnect(&MMCD1)) 
  {
    chprintf(chp, "SD CARD FAILED\r\n");
    return;
  } 
  chprintf(chp, "SD CARD SUCCEEDED\r\n");

  chprintf(chp, "Starting SDC benchmark with %d iterrations.\r\n",it);
  chprintf(chp, "   # | Mount time | f_open time | f_write time | f_close time | umount time | error\r\n");
  chprintf(chp, "-----|------------|-------------|--------------|--------------|-------------|------\r\n");

  for(uint8_t ctr = 0; ctr < it; ctr++)
  {
    error = false;
    chprintf(chp, " %3d |", ctr);

    time = chVTGetSystemTime();
    if(f_mount(&SDC_FS, "/", 1) != FR_OK) error = true;
    time = chVTGetSystemTime() - time;
    chprintf(chp, " %10d |", (uint32_t)ST2US(time));

    time = chVTGetSystemTime();
    if(f_open(&fp, "test.txt", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) error = true;
    time = chVTGetSystemTime() - time;
    chprintf(chp, " %11d |", (uint32_t)ST2US(time));

    time = chVTGetSystemTime();
    size = strlen("abcdefghijklmnopqrstuvwxyz0123456789");
    if(f_write(&fp, "abcdefghijklmnopqrstuvwxyz0123456789", size, &size) != FR_OK) error = true;
    time = chVTGetSystemTime() - time;
    chprintf(chp, " %12d |", (uint32_t)ST2US(time));

    time = chVTGetSystemTime();
    if(f_close(&fp) != FR_OK) error = true;
    time = chVTGetSystemTime() - time;
    chprintf(chp, " %12d |", (uint32_t)ST2US(time));

    time = chVTGetSystemTime();
    if(f_mount(NULL, "/", 0) != FR_OK) error = true;
    time = chVTGetSystemTime() - time;
    chprintf(chp, " %11d | %d\r\n", (uint32_t)ST2US(time), error);
  }

  if(!wasMounted) sdcMount();
}


/** @} */
