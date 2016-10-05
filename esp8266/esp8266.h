
#ifndef _ESP8266_H
#define _ESP8266_H

#include "hal.h"

void espStart(void);

int espInit(void);

bool espHasIP(void);

void espTerm(char* str);

void espRead(void);

#endif 
// _ESP8266_H