//Note the seconds on the transmitter side is being dumb


#include "reciever.hpp"


#define RF95_FREQ 915.0     //Standard frequency for the US
#define RFM95_RST     14    // Marked as RST on the LoRa Radio. This is the reset pin
#define RFM95_CS      10   // Marked as CS on the LoRa Radio. This is the chip select
#define RFM95_INT     4   //Marked as G0 on the LoRa Radio. This is the interrupt pin
#define LED 13


void SerialMonitorSetup(){
//Serial Monitor Setup
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
  delay(100); 
}


void RecieverTest(RH_RF95 *rf95){
  Serial.println("Feather LoRa RX Test!");
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  
  while (!rf95->init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  if (!rf95->setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); 
  Serial.println(RF95_FREQ);
  
}

// Hard coding the size and bounds because we will send the information in the same format every time
//It goes Lat, Long, Speed, vehicle ID
void ParseBuffer(uint8_t buffer[], Vehicle_Info* result) {
  uint8_t offset = 0;
  xSemaphoreTake(vehicleDataMutex, portMAX_DELAY);
  memcpy((&result->latitude), buffer + offset, sizeof(result->latitude));
  offset += sizeof(result->latitude);
  memcpy((&result->longitude), buffer + offset, sizeof(result->longitude));
  offset += sizeof(result->longitude);
  memcpy((&result->speed), buffer + offset, sizeof(result->speed));
  offset += sizeof(result->speed);
  memcpy((&result->vehicle_id), buffer + offset, sizeof(result->vehicle_id));
  offset += sizeof(result->vehicle_id);
  memcpy((&result->transition), buffer + offset, sizeof(result->transition));
  xSemaphoreGive(vehicleDataMutex);
  //Set the cell data and traffic data bits. Alerts the traffic module
  xEventGroupSetBits(rfEventGroup, (rfEventFlagsEnum::updateCellData | rfEventFlagsEnum::updateTrafficData));
  return;
}


//This function is assuming the buffer is the same form as specified in the function above
void PrintBuff(Vehicle_Info* buff){
  Serial.print("Latitude: ");
  Serial.print(buff->latitude, 14);
  Serial.print("   ");
  Serial.print("Longitude: ");
  Serial.print(buff->longitude, 14);
  Serial.println();

  Serial.print("Speed: ");
  Serial.print(buff->speed);
  Serial.println();

  Serial.print("VehicleID: ");
  Serial.print(buff->vehicle_id);
  Serial.println();
  Serial.println("--------------------------------------------------------------------");


}

void RF_Task(void* p_arg){  
      // Setup RF
      RH_RF95 rf95(RFM95_CS, RFM95_INT);
      RecieverTest(&rf95);
      uint8_t data[DataBufferSize];
      uint8_t len = sizeof(data);
      while(1){
            //Serial.println("RF Task!");

            if(rf95.recv(data, &len)){
              ParseBuffer(data, &vehicleData);
              PrintBuff(&vehicleData);
            }
            vTaskDelay(x100ms);
      }
}