

#ifndef __WIFIAPP_H
#define __WIFIAPP_H


/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef enum {
    WIFI_DEINIT = 0,
    WIFI_INITIALIZED,
    WIFI_CONNECTED,
    WIFI_START_TRANSMISSION
} wifi_state;


/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void wifiStart(void);

void wifiTransmitRaw(char* buf, int len);

void wifiAddAP(const char * ssid, const char * password);

#endif