
// TaskHandle_t testTask2;

//void IRAM_ATTR onTimer(){
//  Serial.println("A minute has passed");
//}
//t_idle: idles the system for both cores when nothing is happening
// void Task1_Handler(void * arg){
//  Serial.println("Inside Task1");
//  while(1){
//    Serial.println("task1 running");
//    vTaskDelay(1);
//  } 
// }
// void Task2_Handler(void * arg){
//  Serial.println("Inside Task2");
//  for(;;){
//    Serial.println("task2 running");
//    vTaskDelay(1);
//  } 
// }
//
//void setup() {
//   
//}
//void loop() {
//    Serial.print("test ");
//    Serial.println(__FILE__);
//    delay(1000);
//}

#include <FreeRTOS.h>
#include <task.h>
#include <Arduino.h>

#include "reciever.cpp"
#include "SourDough_Cellular.cpp"


void setup(){

     Serial.begin(115200); 

    TaskHandle_t gpsTask;
    TaskHandle_t rfTask;
    TaskHandle_t micTask;
//Create component tasks
//CORE 0:
//    xTaskCreatePinnedToCore(
//                    &Cellular_Task,   /* Task function. */
//                    "Cellular Task",     /* name of task. */
//                    10240,       /* Stack size of task */
//                    NULL,        /* parameter of the task */
//                    10,           /* priority of the task */
//                    &gpsTask,      /* Task handle to keep track of created task */
//                    0);          /* pin task to core 1 */ 
   xTaskCreatePinnedToCore(
                   &RF_Task,   /* Task function. */
                   "RF Task",     /* name of task. */
                   10240,       /* Stack size of task */
                   NULL,        /* parameter of the task */
                   10,           /* priority of the task */
                   &rfTask,      /* Task handle to keep track of created task */
                   0);          /* pin task to core 1 */
   

}

void loop(){
    delay(10);
}

//    My_timer = timerBegin(0, 80, true);
//    timerAttachInterrupt(My_timer, &onTimer, true);
//    timerAlarmWrite(My_timer, 10 * 1000000, true);  