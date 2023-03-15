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

const double kEarthRadiusFeet = 20902231.96; // Earth's radius in feet
//Returns distance between intersection and approaching vehicle in feet
float Intersection::calculateDistance(float vehicleLat, float vehicleLong) {
      //NOTE: intersection lat and long can be accessed through _latitude and _longitude 
      //Paste distance calculations 
      double delta_lat = DEG_TO_RADS(_latitude) - DEG_TO_RADS(vehicleLat);
      double delta_lon = DEG_TO_RADS(_longitude) - DEG_TO_RADS(vehicleLong);

      double a = pow(sin(delta_lat / 2), 2) + cos(DEG_TO_RADS(vehicleLat)) * cos(DEG_TO_RADS(_latitude)) * pow(sin(delta_lon / 2), 2);
      double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return kEarthRadiusFeet * c;
}

//TODO: finish converting to our needs
float Intersection::calculateBearing(float vehicleLat, float vehicleLong){
      double deltaLong = DEG_TO_RADS(vehicleLong) - DEG_TO_RADS(_longitude);
      double X = cos(DEG_TO_RADS(vehicleLat)) * (sin(deltaLong));
      double Y = cos(DEG_TO_RADS(_latitude)) * sin(DEG_TO_RADS(vehicleLat) - DEG_TO_RADS(_latitude)) * cos(DEG_TO_RADS(vehicleLat)) * cos(deltaLong);
      double bearing = atan2(X,Y);
      if (bearing < 0){
            bearing = (2 * M_PI) + bearing;
      }
      if(bearing >= 5.5 || bearing < 0.78){
            return 'N';
      }
      else if ( bearing >= 0.78 || bearing < 2.35) {
            return 'E';
      }
      else if (2.35 <= bearing && bearing < 3.92){
            return 'S';
      }
      else{
            return 'W';
      }

      //Chapt gpt bearing ranges in degrees
//           if (bearing < 0) {
//         bearing += 360.0;
//     }

//     if ((bearing >= 0 && bearing <= 22.5) || (bearing > 337.5 && bearing <= 360)) {
//         return "north";
//     }
//     else if (bearing > 22.5 && bearing <= 67.5) {
//         return "northeast";
//     }
//     else if (bearing > 67.5 && bearing <= 112.5) {
//         return "east";
//     }
//     else if (bearing > 112.5 && bearing <= 157.5) {
//         return "southeast";
//     }
//     else if (bearing > 157.5 && bearing <= 202.5) {
//         return "south";
//     }
//     else if (bearing > 202.5 && bearing <= 247.5) {
//         return "southwest";
//     }
//     else if (bearing > 247.5 && bearing <= 292.5) {
//         return "west";
//     }
//     else if (bearing > 292.5 && bearing <= 337.5) {
//         return "northwest";
//     }
// }
}

void Intersection::setThreshold(){
      SpeedLimitCycleTime cycleTime = getCycleTime(); //ms
      float minDistanceThreshold = MPH_TO_FPMS(approachVehicle.speed) * static_cast<float>(cycleTime); //mph -> ft/ms * ms = ft
      _startCycleThreshold = minDistanceThreshold + (minDistanceThreshold * SAFETY_FACTOR);      
}

void Intersection::cycleToRed(uint32_t transitionTime, IntersectionState newState){
      switch(newState){
            case IntersectionState::NORTH_SOUTH:
                  //Turn the north and south lights yellow
                  north->setCurrentState(TrafficLightState::YELLOW_LIGHT);
                  south->setCurrentState(TrafficLightState::YELLOW_LIGHT);
                  break;
            case IntersectionState::EAST_WEST:
                  //Turn the east and west lights yellow
                  east->setCurrentState(TrafficLightState::YELLOW_LIGHT);
                  west->setCurrentState(TrafficLightState::YELLOW_LIGHT);
                  break;
            default:
            Serial.println("IN CYCLE TO RED NOT VALID INTERSECTION STATE");
            break;
      }
      
      //This function pretty much starts the timer which will post a sem for the task
      xSemaphoreTake(LightSemaphore, ( TickType_t ) 10);
      xTimerChangePeriod(LightTimer, transitionTime/portTICK_PERIOD_MS, 0);
      xTimerStart( LightTimer, 0 );
      return;
}

void Intersection::changeTrafficDirection(){
      SpeedLimitCycleTime cycleTime = getCycleTime();
      switch(_currentState){
            case IntersectionState::NORTH_SOUTH:
            cycleToRed(static_cast<uint32_t>(cycleTime), IntersectionState::NORTH_SOUTH);
            _originalState = _currentState; //Save traffic orientation to return to upon exiting safeguard
                  break;
            case IntersectionState::EAST_WEST:
            cycleToRed(static_cast<uint32_t>(cycleTime), IntersectionState::EAST_WEST);
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

void vTimerCallback( TimerHandle_t pxTimer ){
      Serial.println("Ay my slime I'm in the callback!!!!");
      xSemaphoreGive(LightSemaphore);
}


void Traffic_Task(void* p_arg){
      //Create Intersection
      constexpr float intersectionLatitude = 40.000113;
      constexpr float intersectionLongitude = -105.236410;
      static Intersection intersection(IntersectionState::NORTH_SOUTH, intersectionLatitude, intersectionLongitude); //only create once

      //Create Timer
      LightTimer =  CreateTimer();
      LightSemaphore = xSemaphoreCreateBinary();

      EventBits_t eventFlags;
      TrafficState trafficState{TrafficState::CHECK_THRESHOLD}; //Check threshold first
      while(1){ //Fatty state machine
            
            eventFlags = xEventGroupWaitBits(rfEventGroup, (updateTrafficData | HomieValid), pdFALSE, pdFALSE, portMAX_DELAY);
            //Update a copy of the vehicle data
            if(updateTrafficData & eventFlags){
                  //Take mutex
                  xSemaphoreTake(vehicleDataMutex, portMAX_DELAY);
                  intersection.approachVehicle = {
                        .latitude = vehicleData.latitude,
                        .longitude = vehicleData.longitude,
                        .speed = vehicleData.speed,
                        .vehicle_id = vehicleData.vehicle_id
                  };
                  //Release Mutex
                  xSemaphoreGive(vehicleDataMutex);

                  //Update distance and bearing
                  intersection.approachVehicle.distance = intersection.calculateDistance(intersection.approachVehicle.latitude, intersection.approachVehicle.longitude);
                  intersection.approachVehicle.bearing = intersection.calculateBearing(DEG_TO_RADS(intersection.approachVehicle.latitude), DEG_TO_RADS(intersection.approachVehicle.longitude));
                  //Clear updateTrafficData flag
                  xEventGroupClearBits(rfEventGroup, updateTrafficData); 
            }

            //Wait on homie valid (verified by Cellular)
            //TODO: NEED A WAY TO VERIFY IF VEHICLE IS APPROACHING OR EXITING
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
                        //TODO: may need to change this logic depending on the bearing logic, relative to the car or intersection??
                        case TrafficState::QUEUE_LIGHT:{ //Queue a light change based on its bearing (heading direction)
                              IntersectionState currentState = intersection.getCurrentState();
                              switch(intersection.approachVehicle.bearing){
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
                              xSemaphoreTake(LightSemaphore, portMAX_DELAY);
                              //North/south are changing transitioning
                              if((intersection.north->getCurrentState() == TrafficLightState::YELLOW_LIGHT) && (intersection.south->getCurrentState() == TrafficLightState::YELLOW_LIGHT)){
                                    intersection.north->setCurrentState(TrafficLightState::RED_LIGHT);
                                    intersection.south->setCurrentState(TrafficLightState::RED_LIGHT);
                                    //Set oncoming to green
                                    intersection.west->setCurrentState(TrafficLightState::GREEN_LIGHT);
                                    intersection.east->setCurrentState(TrafficLightState::GREEN_LIGHT);

                                    intersection.setCurrentState(IntersectionState::EAST_WEST);
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

                                    intersection.setCurrentState(IntersectionState::NORTH_SOUTH);
                              }else{
                                    Serial.println("E/W not transitioning");
                              }
                              xSemaphoreGive(LightSemaphore);
                              trafficState = TrafficState::EXIT_SAFEGUARD;
                              break;
                        }
                        case TrafficState::EXIT_SAFEGUARD: { //Checks that we have exited the intersection 
                              //TODO: Need to calculate if a vehicle is exiting before clearing the flag
                              trafficState = TrafficState::CHECK_THRESHOLD; //reset
                              xEventGroupClearBits(vehicleID_Valid, HomieValid);
                        }
                  }
                              

            }
                 

                  


            }

            vTaskDelay(x100ms);
      
            
            
      }