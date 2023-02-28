#ifndef COMMON_HG
#define COMMON_HG

#include <FreeRTOS.h>
#include <message_buffer.h>
#include <event_groups.h>

//This buffer will be used to send the RF recieved data to the Cellular task to actually send that bitch
MessageBufferHandle_t xMessageBuffer;

const size_t xMessageBufferSizeBytes = 30;
const int DataBufferSize = 14;
const int RecieveBufferSize = 4;

const TickType_t x100ms = pdMS_TO_TICKS( 100 );

EventGroupHandle_t rfEventGroup;
enum rfEventFlagsEnum {
    updateCellData = 0b1 << 0,
    updateTrafficData = 0b1 << 1
};

EventGroupHandle_t vehicleID_Valid;
enum vehicleIDEnum {
    HomieValid = 0b1 << 0,
};

struct Vehicle_Info{
    float latitude;
    float longitude;
    float speed;
    uint16_t vehicle_id;
};

//We want all the tasks to be able to access this because this is shared data among the 3 tasks

EventGroupHandle_t EventGroupCreate();
Vehicle_Info recievebuffer;

static SemaphoreHandle_t recieveMutex; 


#endif