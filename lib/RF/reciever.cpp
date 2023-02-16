//Note the seconds on the transmitter side is being dumb


#include "reciever.hpp"
#include "../common.h"


#define RF95_FREQ 915.0     //Standard frequency for the US
#define RFM95_RST     5    // Marked as RST on the LoRa Radio. This is the reset pin
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
//It goes Lat, Long, Speed, Angle, Second, Minute, Hour, Day, Month, Year
float * ParseBuffer(uint8_t buffer[], float* result, int size) {
    uint8_t temparr[4];
    float temp = 0;
    int oof = 0;
    int bounds[10] = {4, 4, 4, 4, 4, 1, 1, 1, 1, 1};
    int count = 0;
    int totalcnt = 0;

    for(int j = 0; j < sizeof(size); j++){

        for(int i = 0; i < bounds[count]; i++){
            temparr[i] = buffer[i + totalcnt];
            if(bounds[count] == 1){
              (int)buffer[i + totalcnt];
            }
        }

        if(bounds[count] == 1){
            oof = (int)temparr[0];
            temp = (float)oof;
        }

        else{
        temp = *(float*)temparr;
        }

        result[j] = temp;

        for(int i = 0; i < 4; i++){
            temparr[i] = 0;
        }
        count++;

        totalcnt = totalcnt + bounds[j];
    }

    return result;
}


//This function is assuming the buffer is the same form as specified in the function above
void PrintBuff(float* buff){
  Serial.print("Latitude: ");
  Serial.print(buff[0]);
  Serial.print("   ");
  Serial.print("Longitude: ");
  Serial.print(buff[1]);
  Serial.println();

  Serial.print("Speed: ");
  Serial.print(buff[2]);
  Serial.println();

  Serial.print("Angle: ");
  Serial.print(buff[3]);
  Serial.println();

  Serial.print("TimeStamp: ");
  Serial.print(buff[8]);
  Serial.print("/");
  Serial.print(buff[7]);
  Serial.print("/");
  Serial.print(buff[9]);
  Serial.print("     ");
  Serial.print(buff[6]);
  Serial.print(": ");
  Serial.print(buff[5]);
  Serial.print(": ");
  Serial.println(buff[4]);
  Serial.println("--------------------------------------------------------------------");

}

void RF_Task(void* p_arg){  
      // Setup RF
      RH_RF95 rf95(RFM95_CS, RFM95_INT);
      RecieverTest(&rf95);
      uint8_t data[DataBufferSize];
      float recievebuffer[RecieveBufferSize];
      uint8_t len = sizeof(data);
      while(1){
            Serial.println("RF Task!");

            if(rf95.recv(data, &len)){
            ParseBuffer(data, recievebuffer, sizeof(recievebuffer));
            PrintBuff(recievebuffer);
            }
            delay(100);
      }
}