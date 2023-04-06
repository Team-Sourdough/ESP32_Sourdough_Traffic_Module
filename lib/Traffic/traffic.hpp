#ifndef TRAFFIC_HG
#define TRAFFIC_HG
#include "../common.h"
#include <memory>

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

//Traffic Light Pin Defines
#define NORTH_RED 20
#define NORTH_YELLOW 21
#define NORTH_GREEN 35

#define SOUTH_RED 36
#define SOUTH_YELLOW 37
#define SOUTH_GREEN 38

#define EAST_RED 39
#define EAST_YELLOW 40
#define EAST_GREEN 41

#define WEST_RED 42
#define WEST_YELLOW 47
#define WEST_GREEN 48

#define DEG_TO_RADS(angle_degrees) ((angle_degrees) * M_PI / 180.0)
#define MPH_TO_FPMS(mph) (mph * 0.00146667)

//Easy on/off calls for lights
#define ON(light) digitalWrite(light, HIGH);
#define OFF(light) digitalWrite(light, LOW);

#define SPEED_LIMIT 50
#define SAFETY_FACTOR 4 //200% of min threshold will be added 

enum class SpeedLimitCycleTime { //Times in ms
    THIRTY_FIVE_MPH = 3000,
    FIFTY_FIVE_MPH = 5000,
    SEVENTY_FIVE_MPH = 7000,
    UPPER_LIMIT_MPH = 10000
};

SpeedLimitCycleTime getCycleTime(){
    if(SPEED_LIMIT >= 0 && SPEED_LIMIT <= 35){
        return SpeedLimitCycleTime::THIRTY_FIVE_MPH;
    }else if(SPEED_LIMIT > 35 && SPEED_LIMIT <= 55){
        return SpeedLimitCycleTime::FIFTY_FIVE_MPH;
    }else if(SPEED_LIMIT > 55 && SPEED_LIMIT <= 75){
        return SpeedLimitCycleTime::SEVENTY_FIVE_MPH;
    }else if(SPEED_LIMIT > 35 && SPEED_LIMIT <= 55){
        return SpeedLimitCycleTime::UPPER_LIMIT_MPH;
    }
}


enum class TrafficLightState {
    UNKNOWN = 0,
    RED_LIGHT = 1,
    YELLOW_LIGHT = 2,
    GREEN_LIGHT = 3
};

class TrafficLight {
    public:
        TrafficLight(int red, int yellow, int green) : _redLight(red), _yellowLight(yellow), _greenLight(green) {} //initialize member vars
        ~TrafficLight() = default;

        void setCurrentState(TrafficLightState newState);
        TrafficLightState getCurrentState(){
            return _currentState;
        }

    private:
        int _redLight;
        int _yellowLight;
        int _greenLight;

        TrafficLightState _currentState{TrafficLightState::UNKNOWN}; //0 will be checked as a non-valid value (must be initialized at some point to red or green)
};

enum class IntersectionState { //Defines green lights configuration
    UNKNOWN = 0,
    NORTH_SOUTH = 1,
    EAST_WEST = 2
};

class Intersection {
    public:
        Intersection(IntersectionState startState, float latitude, float longitude);
        ~Intersection() = default;

        void cycleToRed(uint32_t transitionTime, IntersectionState newState);
        void updateTransitionInfo(); //threshold (m/s), cycletime (ms)
        float calculateDistance(float vehicleLat, float vehicleLong);
        char calculateBearing(float vehicleLat, float vehicleLong);
        void changeTrafficDirection();
        void holdCurrentDirection(); //TODO: Implement

        void setThreshold(); 
        int getThreshold(){
            return _startCycleThreshold;
        }

        void setCurrentState(IntersectionState newState){
            _currentState = newState;
        }
        IntersectionState getCurrentState(){
            return _currentState;
        }

        void setOriginalState(IntersectionState newState){
            _originalState = newState;
        }
        IntersectionState getOriginalState(){
            return _originalState;
        }

        Vehicle_Info approachVehicle;

        std::unique_ptr<TrafficLight> north;
        std::unique_ptr<TrafficLight> south;
        std::unique_ptr<TrafficLight> west;
        std::unique_ptr<TrafficLight> east;

    private: 
        float _latitude;
        float _longitude;
        int _startCycleThreshold{0};
        IntersectionState _currentState{IntersectionState::UNKNOWN};
        IntersectionState _originalState{IntersectionState::UNKNOWN};

         

};

enum class TrafficState {
    NOP = 0,
    CHECK_THRESHOLD = 1,
    QUEUE_LIGHT = 2,
    SAFEGUARD = 3,
    EXIT_SAFEGUARD = 4,

};


void Traffic_Task(void* p_arg);
void setupTrafficLights();


#endif