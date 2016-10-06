
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include "chthreads.h"
#include "shell.h"

#include "chprintf.h"

#include "usbcfg.h"
#include "usbcdc.h"

#include "esp8266.h"
#include "wifichannel.h"
#include "wifiapp.h"


#define usb_lld_connect_bus(usbp)
#define usb_lld_disconnect_bus(usbp)

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(512)//THD_WA_SIZE(512)

#define THD_STATE_NAMES                                                     \
  "READY", "CURRENT", "SUSPENDED", "WTSEM", "WTMTX", "WTCOND", "SLEEPING",  \
  "WTEXIT", "WTOREVT", "WTANDEVT", "SNDMSGQ", "SNDMSG", "WTMSG", "WTQUEUE", \
  "FINAL"

//  Virtual serial port over USB.
// SerialUSBDriver SDU1;

static thread_t *shelltp = NULL;

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
	size_t n, size;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: mem\r\n");
    return;
  }
  n = chHeapStatus(NULL, &size);
  chprintf(chp, "core free memory : %u bytes\r\n", chCoreGetStatusX());
  chprintf(chp, "heap fragments   : %u\r\n", n);
  chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
	static const char *states[] = {CH_STATE_NAMES};
  thread_t *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: threads\r\n");
    return;
  }
  chprintf(chp, "    addr    stack prio refs     state name\r\n");
  tp = chRegFirstThread();
  do {
    chprintf(chp, "%08lx %08lx %4lu %4lu %9s %s\r\n",
            (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
            (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
            states[tp->p_state], chRegGetThreadNameX(tp));
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}

static void cmd_initesp(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  wifiStart();
}
static void cmd_espterm(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  espTerm(argv[0]);
}

static void cmd_espConnect(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  // espRead();
  esp8266Connect(atoi(argv[0]),argv[1],atoi(argv[2]),ESP_TCP);
}
static void cmd_espStatus(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  // chprintf(chp, "Status=%d\r\n", esp8266GetIpStatus(NULL,NULL));
}
static void cmd_esptestsend(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  // esp8266SendLine(atoi(argv[0]),argv[1]);
  esp8266SendHeader(atoi(argv[0]),strlen(argv[1]));
  esp8266Send(argv[1], true);
}
static void cmd_espClose(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  esp8266Disconnect(atoi(argv[0]));
}

static void cmd_espReadHeader(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  int param = 0;
  int channel = 0;
  char buf[100];
  esp8266ReadRespHeader(&channel, &param, 5);
  esp8266Read(buf,param);
}

static void cmd_wificonnect(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  int chan = channelOpen(ESP_TCP);
  channelConnect(chan, argv[0], atoi(argv[1]));

}
static void cmd_wifisend(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  channelSendLine(0, argv[0]);
}
static void cmd_wifistatus(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  channelStatus(chp);
}


static void cmd_espAddAP(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  if(argc==1)
  {
    wifiAddAP(argv[0],"");
  }
  else if(argc==2)
  {
    wifiAddAP(argv[0],argv[1]);  
  }
}


static const ShellCommand commands[] = {
		{"mem", cmd_mem},
		{"threads", cmd_threads},
    {"espinit", cmd_initesp},
    {"esp", cmd_espterm},
    {"espconnect", cmd_espConnect},
    {"espaddap", cmd_espAddAP},
    {"espstatus", cmd_espStatus},
    {"esptestsend", cmd_esptestsend},
    {"espclose", cmd_espClose},
    {"espreadheader", cmd_espReadHeader},
    {"wificonnect", cmd_wificonnect},
    {"wifisend", cmd_wifisend},
    {"wifistatus", cmd_wifistatus},

		{NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
		(BaseSequentialStream *)&SDU1,
		commands
};

void usbcdcHandleShell(void) {
	if (!shelltp && (SDU1.config->usbp->state == USB_ACTIVE))
      shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
    else if (chThdTerminatedX(shelltp)) {
      chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
      shelltp = NULL;           /* Triggers spawning of a new shell.        */
    }
}

void usbcdcInit(void) {
  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);
  
  /*
   * Shell manager initialization.
   */
  shellInit();
}
