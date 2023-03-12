#include "traffic.hpp"

void setupTrafficLights(){
      pinMode(NORTH_RED, OUTPUT);
      pinMode(NORTH_YELLOW, OUTPUT);
      pinMode(NORTH_GREEN, OUTPUT);
      pinMode(SOUTH_RED, OUTPUT);
      pinMode(SOUTH_YELLOW, OUTPUT);
      pinMode(SOUTH_GREEN, OUTPUT);
      pinMode(EAST_RED, OUTPUT);
      pinMode(EAST_YELLOW, OUTPUT);
      pinMode(EAST_GREEN, OUTPUT);
      pinMode(WEST_RED, OUTPUT);
      pinMode(WEST_YELLOW, OUTPUT);
      pinMode(WEST_GREEN, OUTPUT);

      // ON(NORTH_RED);
      // ON(NORTH_YELLOW);
      // ON(NORTH_GREEN);
      // ON(SOUTH_RED);
      // ON(SOUTH_YELLOW);
      // ON(SOUTH_GREEN);
      // ON(EAST_RED);
      // ON(EAST_YELLOW);
      // ON(EAST_GREEN);
      // ON(WEST_RED);
      // ON(WEST_YELLOW);
      // ON(WEST_GREEN);
}

//------------------------TRAFFIC LIGHT CLASS DEFINES---------------------------------------------------//
void TrafficLight::setCurrentState(TrafficLightState newState){
      _currentState = newState;

      switch(_currentState){ //Turn appropriate lights on/off
            case TrafficLightState::GREEN_LIGHT:
                  ON(_greenLight);
                  OFF(_yellowLight);
                  OFF(_redLight);
                  break;
            case TrafficLightState::YELLOW_LIGHT:
                  OFF(_greenLight);
                  ON(_yellowLight);
                  OFF(_redLight);
                  break;
            case TrafficLightState::RED_LIGHT:
                  OFF(_greenLight);
                  OFF(_yellowLight);
                  ON(_redLight);
                  break;
            default:
                  Serial.println("Unknown traffic light state");
                  break;

      }
}
void TrafficLight::getCurrentState(TrafficLightState *currentState){
      *currentState = _currentState;
} 

//------------------------INTERSECTION CLASS DEFINES---------------------------------------------------//
//Constructs an intersection object and creates all the traffic light sub classes
Intersection::Intersection(IntersectionState startState, float latitude, float longitude) : _currentState(startState), _latitude(latitude), _longitude(longitude){
      setupTrafficLights(); //Sets pins as OUTPUT
      _north = make_unique<TrafficLight>(NORTH_RED, NORTH_YELLOW, NORTH_GREEN); 
      _south = make_unique<TrafficLight>(SOUTH_RED, SOUTH_YELLOW, SOUTH_GREEN);
      _east =  make_unique<TrafficLight>(EAST_RED, EAST_YELLOW, EAST_GREEN);
      _west =  make_unique<TrafficLight>(WEST_RED, WEST_YELLOW, WEST_GREEN);

      if(startState == IntersectionState::NORTH_SOUTH){ //North/south starting as green light
            _north->setCurrentState(TrafficLightState::GREEN_LIGHT);
            _south->setCurrentState(TrafficLightState::GREEN_LIGHT);
            _east->setCurrentState(TrafficLightState::RED_LIGHT);
            _west->setCurrentState(TrafficLightState::RED_LIGHT);
      }else if(startState == IntersectionState::EAST_WEST){ //East/West starting as green light
            _north->setCurrentState(TrafficLightState::RED_LIGHT);
            _south->setCurrentState(TrafficLightState::RED_LIGHT);
            _east->setCurrentState(TrafficLightState::GREEN_LIGHT);
            _west->setCurrentState(TrafficLightState::GREEN_LIGHT);
      }else{
            Serial.println("Unknown intersection state");
      }
}

float Intersection::calculateDistance(float vehicleLat, float vehicleLong) {
      //NOTE: intersection lat and long can be accessed through _latitude and _longitude 
      //Paste distance calculations 

}

void Intersection::changeTrafficDirection(){
      SpeedLimitCycleTime cycleTime = getCycleTime();
      switch(_currentState){
            case IntersectionState::NORTH_SOUTH:
                  break;
            case IntersectionState::EAST_WEST:
                  break;
            default:
                  Serial.println("Unknown intersection state");
                  break;
      }
}



void Traffic_Task(void* p_arg){
      constexpr float intersectionLatitude = 40.000113;
      constexpr float intersectionLongitude = -105.236410;
      static Intersection intersection(IntersectionState::NORTH_SOUTH, intersectionLatitude, intersectionLongitude); //only create once

      EventBits_t eventFlags;
      TrafficState trafficState{TrafficState::CHECK_THRESHOLD}; //Check threshold first
      while(1){ //Fatty state machine
            
            eventFlags = xEventGroupWaitBits(rfEventGroup, (updateTrafficData | HomieValid), pdFALSE, pdFALSE, portMAX_DELAY);
            //Update a copy of the vehicle data
            if(updateTrafficData & eventFlags){
                  //Take mutex

                  intersection.approachVehicle = {
                        .latitude = vehicleData.latitude,
                        .longitude = vehicleData.longitude,
                        .speed = vehicleData.speed,
                        .vehicle_id = vehicleData.vehicle_id
                  };
                  //Release Mutex

                  intersection.approachVehicle.distance = intersection.calculateDistance(intersection.approachVehicle.latitude, intersection.approachVehicle.longitude);
                  intersection.approachVehicle.bearing = intersection.calculateBearing(intersection.approachVehicle.latitude, intersection.approachVehicle.longitude);
                  //Clear updateTrafficData flag

            }
            //Wait on homie valid (verified by Cellular)
            if(HomieValid & eventFlags){
                  switch(trafficState){
                        case TrafficState::CHECK_THRESHOLD: {
                              int threshold = intersection.getThreshold();
                              if(threshold == 0){
                                    //Set Threshold if it hasn't already been set
                                    intersection.setThreshold();
                              }
                              //Check current distance against threshold
                              if(intersection.approachVehicle.distance <= threshold){
                                    //Start light cycle transisiton
                                    trafficState = TrafficState::QUEUE_LIGHT;
                              }
                              break;
                        }
                        case TrafficState::QUEUE_LIGHT:{
                              //QUEUE LIGHT CHANGE - STATE = QUEUE_LIGHT
                              switch(intersection.approachVehicle.bearing){
                                    IntersectionState currentState{IntersectionState::UNKNOWN};
                                    intersection.getCurrentState(&currentState);
                                    case 'N':
                                    case 'S':
                                          if(currentState == IntersectionState::EAST_WEST){
                                                //Need to change to a N/S configuration
                                                intersection.changeTrafficDirection();
                                          }else{
                                                intersection.holdCurrentDirection();
                                          }
                                          break;
                                    case 'E':
                                    case 'W':
                                          if(currentState == IntersectionState::NORTH_SOUTH){
                                                //Need to change to a E/W configuration
                                                intersection.changeTrafficDirection();
                                          }else{
                                                intersection.holdCurrentDirection();
                                          }
                                          break;
                                    default:
                                          Serial.println("Unknown vehicle bearing");
                                          break;
                              }
                              break;
                        }
                        case TrafficState::SAFEGUARD:{
                              //STATE = SAFEGUARD 
                              //PEND TIMER SEMAPHORE
                              //if north == yellow{

                              //}else{ east or west is yellow

                              //}
                              break;
                        }
                  }
                              

            }
                 

                  


            }

            xEventGroupClearBits(vehicleID_Valid, HomieValid);
            vTaskDelay(x100ms);
      
            
            
      }