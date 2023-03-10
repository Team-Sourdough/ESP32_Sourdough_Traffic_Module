#include <FreeRTOS.h>
#include <task.h>
#include <Arduino.h>

#include "reciever.cpp"
#include "SourDough_Cellular.cpp"
#include "traffic.cpp"
#include "../lib/common.h"

#include "../lib/common.c"


void setup(){

     Serial.begin(115200); 

    TaskHandle_t cellTask;
    TaskHandle_t rfTask;
    TaskHandle_t trafficTask;

    rfEventGroup = EventGroupCreate();
    vehicleID_Valid = EventGroupCreate();
    vehicleDataMutex = xSemaphoreCreateMutex();
//Create component tasks
//CORE 0:
   xTaskCreatePinnedToCore(
                   &Cellular_Task,   /* Task function. */
                   "Cellular Task",     /* name of task. */
                   10240,       /* Stack size of task */
                   NULL,        /* parameter of the task */
                   10,           /* priority of the task */
                   &cellTask,      /* Task handle to keep track of created task */
                   0);          /* pin task to core 1 */ 
   xTaskCreatePinnedToCore(
                   &RF_Task,   /* Task function. */
                   "RF Task",     /* name of task. */
                   10240,       /* Stack size of task */
                   NULL,        /* parameter of the task */
                   10,           /* priority of the task */
                   &rfTask,      /* Task handle to keep track of created task */
                   0);          /* pin task to core 1 */
    xTaskCreatePinnedToCore(
                &Traffic_Task,   /* Task function. */
                "Traffic Task",     /* name of task. */
                10240,       /* Stack size of task */
                NULL,        /* parameter of the task */
                10,           /* priority of the task */
                &trafficTask,      /* Task handle to keep track of created task */
                1);          /* pin task to core 1 */

//We should clear all of our flags, for some reason I see that some of them are high before they should be
xEventGroupClearBits(rfEventGroup, (updateCellData | updateTrafficData));
xEventGroupClearBits(vehicleID_Valid, HomieValid);
}

void loop(){
    delay(10);
}

//    My_timer = timerBegin(0, 80, true);
//    timerAttachInterrupt(My_timer, &onTimer, true);
//    timerAlarmWrite(My_timer, 10 * 1000000, true);  