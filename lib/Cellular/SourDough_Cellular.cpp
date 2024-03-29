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

    // Begin connection to Nothub
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


    // Wait for Connected is True message
    const char * status = "connected";
    J *rsp = NULL;
    J *rsp_body = JGetObjectItem(rsp, status);
    const char *json_body = JPrintUnformatted(rsp_body);

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
    return;
}

void Cellular_Send(Notecard *NOTE) {

    // Make local copy of received GPS data
    GPSdata localGPSdata;
    localGPSdata.latitude = vehicleData.latitude;
    localGPSdata.longitude = vehicleData.longitude;
    localGPSdata.speed = vehicleData.speed;
    localGPSdata.vehicle_id = vehicleData.vehicle_id;
    localGPSdata.distance = vehicleData.distance;
    localGPSdata.bearing = vehicleData.bearing;
    localGPSdata.threshold = vehicleData.threshold;

    // Start sending data to server
    J *rsp = NULL;
    J *rsp_body = JGetObjectItem(rsp, "result");
    char *json_body = JPrintUnformatted(rsp_body);
    char *json = JPrintUnformatted(rsp);

    // Wait for either a valid (200) or not-valid (500) response
    while (strcmp(json_body,"200") | strcmp(json_body,"500")) {
        J *req = NoteNewRequest("web.post");
        JAddStringToObject(req, "route", "Sourdough_ping");

        if(req != NULL){
            JAddBoolToObject(req, "sync", true);
            J *body = JAddObjectToObject(req, "body");
            if (body) {
                JAddNumberToObject(body, "VehicleID", localGPSdata.vehicle_id);
                JAddNumberToObject(body, "Latitude", localGPSdata.latitude);
                JAddNumberToObject(body, "Longitude", localGPSdata.longitude);
                JAddNumberToObject(body, "Speed", localGPSdata.speed);
                JAddNumberToObject(body, "LightID", LIGHT_ID);
                JAddNumberToObject(body, "Distance", localGPSdata.distance);
                JAddNumberToObject(body, "Threshold", localGPSdata.threshold);
                JAddNumberToObject(body, "Bearing", localGPSdata.bearing);
            }

            usbSerial.println("----------------------- Waiting for Response -----------------------");
            rsp = NoteRequestResponse(req);
            rsp_body = JGetObjectItem(rsp, "result");
            json_body = JPrintUnformatted(rsp_body);

            EventBits_t eventFlags = (xEventGroupGetBits(rfEventGroup) & 4);
            
            // Valid Vehicle and valid flag not set
            if(!strcmp(json_body,"200")) {
                if(!eventFlags){
                    xEventGroupSetBits(rfEventGroup, HomieValid);
                    usbSerial.print("VERFIED VEHICLE FOUND! Set Valid flag");
                    usbSerial.println(json_body);
                    xEventGroupClearBits(rfEventGroup,updateCellData);
                    return;
                }
                else{
                    usbSerial.print("VERFIED VEHICLE FOUND! Set Valid flag");
                    usbSerial.println(json_body);
                    xEventGroupClearBits(rfEventGroup,updateCellData);
                    return;
                }
            }
            // InValid Vehicle
            else if(!strcmp(json_body,"500")) {
                usbSerial.print("INVALID VEHICLE! Response: ");
                usbSerial.println(json_body);
                xEventGroupClearBits(rfEventGroup,updateCellData);
                return;
            }

            delay(1000);
            }
    }
    return;
}

void Cellular_Task(void* p_arg){  

    Notecard NOTE;
    EventBits_t eventFlags;

    Serial.println("Setup Cellular!");
    Cellular_Setup(&NOTE);
    
    while(1){

        Serial.println("BEFORE WAIT");
        Serial.println(eventFlags);

        eventFlags = xEventGroupWaitBits(rfEventGroup, updateCellData, pdFALSE, pdFALSE, portMAX_DELAY);
        Serial.println("AFTER WAIT");
        Serial.println(eventFlags);

        if(eventFlags){
            Cellular_Send(&NOTE);
        }

        delay(1000);
    }
}