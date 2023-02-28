#ifndef COMMON_HG
#define COMMON_HG

#include <Arduino.h>

//RTOS includes
#include <FreeRTOS.h>
#include <message_buffer.h>
#include <event_groups.h>


#define BUTTON_GPIO 2
// const TickType_t x100ms = pdMS_TO_TICKS( 100 );
//Message Buffer
//--------------------------------------------------------------------------------------------------------------
//This buffer will be used to send the gps data to the RF task to actually send that bitch
MessageBufferHandle_t xMessageBuffer;
//Our buffer size needs to be 25 bytes, however there are an additional 4 bytes to hold the size of the buffer
const size_t xMessageBufferSizeBytes = 30;
const int DataBufferSize = 25;
// uint8_t gpsData[DataBufferSize];

const TickType_t x100ms = pdMS_TO_TICKS( 100 );

//Function Definitions
void Message_Buffer_Recieve(MessageBufferHandle_t xMessageBuffer, uint8_t data_array[]);
MessageBufferHandle_t Message_Buffer_Create_25byte();
void Message_Buffer_Send( MessageBufferHandle_t xMessageBuffer, uint8_t data_array[]);
//--------------------------------------------------------------------------------------------------------------

//Event Group
//--------------------------------------------------------------------------------------------------------------
EventGroupHandle_t cellularEventGroup;
enum cellularEventFlagsEnum {
    sendData = 0b1 << 0,
    verified_found = 0b1 << 1
};

const TickType_t EVENT_GROUP_PEND_BLOCKING = portMAX_DELAY; //pdMS_TO_TICKS( 10 );//100ms //portMAX_DELAY; //Max number of ticks

//Function Definitions
EventGroupHandle_t EventGroupCreate();
//--------------------------------------------------------------------------------------------------------------


#endif