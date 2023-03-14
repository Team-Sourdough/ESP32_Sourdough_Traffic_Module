#include "common.h"

void Message_Buffer_Recieve(MessageBufferHandle_t xMessageBuffer, uint8_t data_array[]){
      size_t xReceivedBytes;
      xReceivedBytes = xMessageBufferReceive( xMessageBuffer, ( void * ) data_array, sizeof(*data_array),x100ms);
      return;
}


//This will create a 16 byte buffer to send speed, lat and long
MessageBufferHandle_t Message_Buffer_Create_12byte(){
      xMessageBuffer = xMessageBufferCreate(xMessageBufferSizeBytes);

      if( xMessageBuffer == NULL ){
            // There was not enough heap memory space available to create the
            // message buffer.
            Serial.println("Something went wrong in the creation of the message buffer");
            return NULL;
      }
      else{
            return xMessageBuffer;
      }

}

//This is the function that will send our data to the data buffer, this will be read in the RF task so we can send the data!
//No output
void Message_Buffer_Send( MessageBufferHandle_t xMessageBuffer, uint8_t data_array[]){
      size_t xBytesSent;

      xBytesSent = xMessageBufferSend(xMessageBuffer, ( void * ) data_array, sizeof(*data_array), x100ms);

      if(xBytesSent != sizeof(*data_array)){
            // The call to xMessageBufferSend() times out before there was enough
            // space in the buffer for the data to be written.
            Serial.println("The buffer did not allocate space in time\n");
            return;
      }
      else{
            
            Serial.println("We good\n");
      }
      return;
}

EventGroupHandle_t EventGroupCreate(){
      // Attempt to create the event group.
      EventGroupHandle_t xCreatedEventGroup = xEventGroupCreate();

      // Was the event group created successfully?
      if( xCreatedEventGroup == NULL )
      {
      // The event group was not created because there was insufficient
      // FreeRTOS heap available.
            while(1); //Get stuck if event group doesn't get created
      }
      else
      {
            return xCreatedEventGroup;
      }
}

TimerHandle_t CreateTimer(void){
      TimerHandle_t xtimer;
      xtimer = xTimerCreate("Light Timer",       // Just a text name, not used by the kernel.
                              100,   // The timer period in ticks.
                              pdFALSE,        // The timers will auto-reload themselves when they expire.
                              ( void * ) 2,  // Assign each timer a unique id equal to its array index.
                              vTimerCallback // Each timer calls the same callback when it expires.
                              );

      if(xtimer == NULL){
            Serial.println("Something went wrong with the Timer Creation");
      }

      return xtimer;
}