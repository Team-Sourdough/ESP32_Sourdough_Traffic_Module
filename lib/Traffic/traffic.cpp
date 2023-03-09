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





void Traffic_Task(void* p_arg){
      constexpr float intersectionLatitude = 40.000113;
      constexpr float intersectionLongitude = -105.236410;
      static Intersection intersection(IntersectionState::NORTH_SOUTH, intersectionLatitude, intersectionLongitude); //only create once
      while(1){ //Fatty state machine
            xEventGroupClearBits(vehicleID_Valid,HomieValid);
            vTaskDelay(x100ms);
      
            
            
      }
}