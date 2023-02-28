#include <SourDough_Cellular.hpp>
#include <string.h>

#define INBOUND_QUEUE_COMMAND_FIELD "my-request-type"
 
void Cellular_Setup(Notecard *NOTE) {


    usbSerial.begin(115200);
    while (!usbSerial) {
    ; // wait for serial port to connect. Needed for native USB
    }
    usbSerial.println("Starting...");
 
    txRxPinsSerial.begin(9600, SERIAL_8N1, 18,17);
    NOTE->begin(txRxPinsSerial, 9600);
    NOTE->setDebugOutputStream(usbSerial);

    J *req = NoteNewRequest("hub.set");
    if (req != NULL) {
        JAddStringToObject(req, "mode", "continuous");
        JAddStringToObject(req, "product", NOTE_PRODUCT_UID);
        JAddBoolToObject(req, "sync", true);
        NoteRequest(req);
    }
    req = NoteNewRequest("hub.sync");
    if (req != NULL) {
        JAddBoolToObject(req, "allow", true);
        NoteRequest(req);
    }

    const char * status = "connected";

    J *rsp = NULL;
    J *rsp_body = JGetObjectItem(rsp, status);
    const char *json_body = JPrintUnformatted(rsp_body);

    usbSerial.print("RESPONSE: Connected is ");

    while (strcmp(json_body,"true")) {
        req = NoteNewRequest("hub.status");
        usbSerial.println("waiting for connection.....");
        rsp = NoteRequestResponse(req);
        rsp_body = JGetObjectItem(rsp, status);
        json_body = JPrintUnformatted(rsp_body);
        
        if(!strcmp(json_body,"true")){
            usbSerial.print("RESPONSE: CONNECTED!");
        }
        else {
            usbSerial.println("RESPONSE: NOT CONNECTED!");
        }

        delay(10000);
    }

    char *json = JPrintUnformatted(rsp);

    usbSerial.print("FULL RESPONSE: ");
    usbSerial.println(json);

    NoteDeleteResponse(rsp);
}

void Cellular_Send(Notecard *NOTE, uint8_t rfDataArray[]) {

    J *rsp = NULL;
    J *rsp_body = JGetObjectItem(rsp, "result");
    char *json_body = JPrintUnformatted(rsp_body);
    char *json = JPrintUnformatted(rsp);

    while (strcmp(json_body,"200") | strcmp(json_body,"500")) {
        J *req = NoteNewRequest("web.post");
        JAddStringToObject(req, "route", "Sourdough_ping");

        if(req != NULL){
            JAddBoolToObject(req, "sync", true);
            J *body = JAddObjectToObject(req, "body");
            if (body) {
                JAddNumberToObject(body, "VehicleID", 12);
                JAddNumberToObject(body, "LightID", 10);
                JAddNumberToObject(body, "x", 10);
                JAddNumberToObject(body, "y", 10);
                JAddStringToObject(body, "Drivers", "Wiggles");
                JAddStringToObject(body, "VehicleType", "Firetruck");
            }

            usbSerial.println("----------------------- Waiting for Response -----------------------");
            rsp = NoteRequestResponse(req);
            rsp_body = JGetObjectItem(rsp, "result");
            json_body = JPrintUnformatted(rsp_body);
            
            if(!strcmp(json_body,"200")) {
                xEventGroupSetBits(cellularEventGroup, verified_found);
                usbSerial.print("VERFIED VEHICLE FOUND! Response: ");
                usbSerial.println(json_body);
            }
            else if(!strcmp(json_body,"500")) {
                usbSerial.print("UNIDENTIFIED VEHICLE! Response: ");
                usbSerial.println(json_body);
            }

            delay(20000);
            }
    }
    NoteDeleteResponse(rsp);
}

void Cellular_Task(void* p_arg){  

    Notecard NOTE;
    bool send_data = 0;
    EventBits_t eventFlags;
    uint8_t rfDataArray[DataBufferSize];

    Serial.println("Setup Cellular!");
    Cellular_Setup(&NOTE);
    
    while(1){
        eventFlags = xEventGroupWaitBits(cellularEventGroup, sendData, pdFALSE, pdFALSE, EVENT_GROUP_PEND_BLOCKING);


        if(send_data){
            Cellular_Send(&NOTE,&rfDataArray[DataBufferSize]);
        }
        delay(10000);
    }
}