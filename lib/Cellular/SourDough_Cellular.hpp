#ifndef SourDough_Cellular
#define SourDough_Cellular

#include <Notecard.h>
#include "../common.h"

#define NOTE_PRODUCT_UID "edu.colorado.sare3396:safeguard"

#define usbSerial Serial //Using Serial1 because Serial is taken for computer
#define txRxPinsSerial Serial1

void Cellular_Setup(Notecard *NOTE);
void Cellular_Send(Notecard *NOTE, uint8_t rfDataArray[]);

void Cellular_Task(void* p_arg);


#endif