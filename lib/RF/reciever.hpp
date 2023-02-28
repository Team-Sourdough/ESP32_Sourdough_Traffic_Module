#ifndef RECIEVER_HG
#define RECIEVER_HG

#include <Arduino.h>
#include <RH_RF95.h>
#include "../common.h"


void SerialMonitorSetup();
void RecieverTest(RH_RF95 *rf95);
void ParseBuffer(uint8_t buffer[], Vehicle_Info result);
void PrintBuff(Vehicle_Info buff);

void RF_Task(void* p_arg);


#endif