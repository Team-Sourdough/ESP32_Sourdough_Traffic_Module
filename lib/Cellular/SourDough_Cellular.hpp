#ifndef SourDough_Cellular
#define SourDough_Cellular

#include <Notecard.h>
#include "../common.h"

#define NOTE_PRODUCT_UID "edu.colorado.sare3396:safeguard"
#define LIGHT_ID 1

#define usbSerial Serial //Using Serial1 because Serial is taken for computer
#define txRxPinsSerial Serial1

void Cellular_Setup(Notecard *NOTE);
void Cellular_Send(Notecard *NOTE);

void Cellular_Task(void* p_arg);

struct GPSdata{
    float latitude;
    float longitude;
    float speed;
    float threshold;
    char* bearing;
    float distance;
    uint16_t vehicle_id;
};

#endif