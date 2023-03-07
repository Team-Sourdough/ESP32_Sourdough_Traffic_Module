#include "traffic.hpp"








void Traffic_Task(void* p_arg){  
       while(1){
            xEventGroupClearBits(vehicleID_Valid,HomieValid);
            vTaskDelay(x100ms);

      }
}