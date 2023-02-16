#ifndef COMMON_HG
#define COMMON_HG

#include <FreeRTOS.h>
#include <message_buffer.h>

//This buffer will be used to send the RF recieved data to the Cellular task to actually send that bitch
MessageBufferHandle_t xMessageBuffer;

const size_t xMessageBufferSizeBytes = 30;
const int DataBufferSize = 25;
const int RecieveBufferSize = 10;

const TickType_t x100ms = pdMS_TO_TICKS( 100 );



#endif