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

//------------------------INTERSECTION CLASS DEFINES---------------------------------------------------//
//Constructs an intersection object and creates all the traffic light sub classes
Intersection::Intersection(IntersectionState startState, float latitude, float longitude) : _currentState(startState), _latitude(latitude), _longitude(longitude){
      setupTrafficLights(); //Sets pins as OUTPUT
      north = make_unique<TrafficLight>(NORTH_RED, NORTH_YELLOW, NORTH_GREEN); 
      south = make_unique<TrafficLight>(SOUTH_RED, SOUTH_YELLOW, SOUTH_GREEN);
      east =  make_unique<TrafficLight>(EAST_RED, EAST_YELLOW, EAST_GREEN);
      west =  make_unique<TrafficLight>(WEST_RED, WEST_YELLOW, WEST_GREEN);

      if(startState == IntersectionState::NORTH_SOUTH){ //North/south starting as green light
            north->setCurrentState(TrafficLightState::GREEN_LIGHT);
            south->setCurrentState(TrafficLightState::GREEN_LIGHT);
            east->setCurrentState(TrafficLightState::RED_LIGHT);
            west->setCurrentState(TrafficLightState::RED_LIGHT);
      }else if(startState == IntersectionState::EAST_WEST){ //East/West starting as green light
            north->setCurrentState(TrafficLightState::RED_LIGHT);
            south->setCurrentState(TrafficLightState::RED_LIGHT);
            east->setCurrentState(TrafficLightState::GREEN_LIGHT);
            west->setCurrentState(TrafficLightState::GREEN_LIGHT);
      }else{
            Serial.println("Unknown intersection state");
      }
}

float Intersection::calculateDistance(float vehicleLat, float vehicleLong) {
      //NOTE: intersection lat and long can be accessed through _latitude and _longitude 
      //Paste distance calculations 

}

//TODO: finish converting to our needs
float Intersection::calculateBearing(Vehicle_Info* vehicleInfo){
      float deltaLong = DEG_TO_RAD(vehicleInfo->longitude) - DEG_TO_RAD(vehicleInfo->prevLong);
      float X = cos(pos2[0]) * (sin(deltaLong));
      float Y = cos(pos1[0]) * sin(pos2[0]) - sin(pos1[0]) * cos(pos2[0]) * cos(deltaLong);
      float bearing = atan2(X,Y);
      if (bearing < 0){
            bearing = (2 * M_PI) + bearing;
      }
      if(bearing >= 5.5 || bearing < 0.78){
            return 'N';
      }
      else if ( bearing >= 0.78 || bearing < 2.35) {
            return 'E';
      }
      else if (2.35 <= bearing < 3.92){
            return 'S';
      }
      else{
            return 'W';
      }
}

void Intersection::changeTrafficDirection(){
      SpeedLimitCycleTime cycleTime = getCycleTime();
      switch(_currentState){
            case IntersectionState::NORTH_SOUTH:
            //TODO: Rebase with Tanners code

            _originalState = _currentState; //Save traffic orientation to return to upon exiting safeguard
                  break;
            case IntersectionState::EAST_WEST:
            //TODO: Rebase with Tanners code

            _originalState = _currentState; //Save traffic orientation to return to upon exiting safeguard
                  break;
            default:
                  Serial.println("Unknown intersection state");
                  break;
      }
}

void Intersection::holdCurrentDirection(){
      _originalState = _currentState; //maintain state when safeguard "exits"
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
                  float prevLat = intersection.approachVehicle.latitude;
                  float prevLong = intersection.approachVehicle.longitude;
                  //Take mutex
                  xSemaphoreTake(vehicleDataMutex, portMAX_DELAY);
                  intersection.approachVehicle = {
                        .latitude = vehicleData.latitude,
                        .prevLat = prevLat,
                        .longitude = vehicleData.longitude,
                        .prevLong = vehicleData.longitude,
                        .speed = vehicleData.speed,
                        .vehicle_id = vehicleData.vehicle_id
                  };
                  //Release Mutex
                  xSemaphoreGive(vehicleDataMutex);

                  //Update distance and bearing
                  intersection.approachVehicle.distance = intersection.calculateDistance(intersection.approachVehicle.latitude, intersection.approachVehicle.longitude);
                  intersection.approachVehicle.bearing = intersection.calculateBearing();
                  //Clear updateTrafficData flag
                  xEventGroupClearBits(rfEventGroup, updateTrafficData); 
            }

            //Wait on homie valid (verified by Cellular)
            if(HomieValid & eventFlags){
                  switch(trafficState){
                        case TrafficState::CHECK_THRESHOLD: { //Check that vehicle has crossed a distance threshold
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
                        case TrafficState::QUEUE_LIGHT:{ //Queue a light change based on its bearing (heading direction)
                              switch(intersection.approachVehicle.bearing){
                                    IntersectionState currentState = intersection.getCurrentState();
                                    case 'N':
                                    case 'S':
                                          if(currentState == IntersectionState::EAST_WEST){
                                                //Need to change to a N/S configuration
                                                intersection.changeTrafficDirection();
                                          }else if(currentState == IntersectionState::NORTH_SOUTH){
                                                intersection.holdCurrentDirection();
                                          }else{
                                                Serial.println("UNKNOWN intersection state");
                                                break;
                                          }
                                          trafficState = TrafficState::SAFEGUARD;
                                          break;
                                    case 'E':
                                    case 'W':
                                          if(currentState == IntersectionState::NORTH_SOUTH){
                                                //Need to change to a E/W configuration
                                                intersection.changeTrafficDirection();
                                          }else if(currentState == IntersectionState::EAST_WEST){
                                                intersection.holdCurrentDirection();
                                          }else{
                                               Serial.println("UNKNOWN intersection state"); 
                                               break;
                                          }
                                          trafficState = TrafficState::SAFEGUARD;
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
                              xSemaphoreTake(timerSemaphore, portMAX_DELAY);
                              //North/south are changing transitioning
                              if((intersection.north->getCurrentState() == TrafficLightState::YELLOW_LIGHT) && (intersection.south->getCurrentState() == TrafficLightState::YELLOW_LIGHT)){
                                    intersection.north->setCurrentState(TrafficLightState::RED_LIGHT);
                                    intersection.south->setCurrentState(TrafficLightState::RED_LIGHT);
                                    //Set oncoming to green
                                    intersection.west->setCurrentState(TrafficLightState::GREEN_LIGHT);
                                    intersection.east->setCurrentState(TrafficLightState::GREEN_LIGHT);
                              }else{
                                    Serial.println("N/S not transitioning");
                              }

                              //East/West changing
                              if((intersection.east->getCurrentState() == TrafficLightState::YELLOW_LIGHT) && (intersection.west->getCurrentState() == TrafficLightState::YELLOW_LIGHT)){
                                    intersection.east->setCurrentState(TrafficLightState::RED_LIGHT);
                                    intersection.west->setCurrentState(TrafficLightState::RED_LIGHT);
                                    //Set oncoming to green
                                    intersection.north->setCurrentState(TrafficLightState::GREEN_LIGHT);
                                    intersection.south->setCurrentState(TrafficLightState::GREEN_LIGHT);
                              }else{
                                    Serial.println("E/W not transitioning");
                              }
                              trafficState = TrafficState::EXIT_SAFEGUARD;
                              break;
                        }
                        case TrafficState::EXIT_SAFEGUARD: { //Checks that we have exited the intersection 
                              //TODO: decide if we need want to transisiton back to "original state" or just leave lights in the current config and start process over?

                              trafficState = TrafficState::CHECK_THRESHOLD; //reset
                        }
                  }
                              

            }
                 

                  


            }

            xEventGroupClearBits(vehicleID_Valid, HomieValid);
            vTaskDelay(x100ms);
      
            
            
      }