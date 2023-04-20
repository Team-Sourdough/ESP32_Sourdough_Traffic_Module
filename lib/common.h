#ifndef COMMON_HG
#define COMMON_HG

#include <FreeRTOS.h>
#include <Arduino.h>
#include <message_buffer.h>
#include <event_groups.h>
#include <semphr.h>
#include <timers.h>

//This buffer will be used to send the RF recieved data to the Cellular task to actually send that bitch
MessageBufferHandle_t xMessageBuffer;

const size_t xMessageBufferSizeBytes = 30;
const int DataBufferSize = 15;
const int RecieveBufferSize = 4;

const TickType_t x100ms = pdMS_TO_TICKS( 100 );

EventGroupHandle_t rfEventGroup;
enum rfEventFlagsEnum {
    updateCellData = 0b1 << 0,
    updateTrafficData = 0b1 << 1,
    HomieValid = 0b1 << 2
};

EventGroupHandle_t vehicleID_Valid;
enum vehicleIDEnum {
    
};

struct Vehicle_Info{
    float latitude;
    float longitude;
    float distance;
    char bearing;
    float speed;
    uint16_t vehicle_id;
    float threshold;
    uint8_t transition;
};

//We want all the tasks to be able to access this because this is shared data among the 3 tasks

EventGroupHandle_t EventGroupCreate();
Vehicle_Info vehicleData;

static SemaphoreHandle_t vehicleDataMutex; 
TimerHandle_t LightTimer;
SemaphoreHandle_t LightSemaphore;
TimerHandle_t CreateTimer(void);


#endif