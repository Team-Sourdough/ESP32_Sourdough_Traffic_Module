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

//Easy on/off calls for lights
#define ON(light) digitalWrite(light, HIGH);
#define OFF(light) digitalWrite(light, LOW);

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

        void cycleToRed(uint32_t transitionTime);
        void cycleToGreen(uint32_t transitionTime);

        void setCurrentState(TrafficLightState newState);
        void getCurrentState(TrafficLightState *currentState);

    private:
        int _redLight;
        int _yellowLight;
        int _greenLight;

        TrafficLightState _currentState{TrafficLightState::UNKNOWN}; //0 will be checked as a non-valid value (must be initialized at some point to red or green)
};

enum class IntersectionState {
    UNKNOWN = 0,
    NORTH_SOUTH = 1,
    EAST_WEST = 2
};

class Intersection {
    public:
        Intersection(IntersectionState startState, float latitude, float longitude);
        ~Intersection() = default;

        void changeTrafficDirection();
        void holdCurrentDirection();

        void setCurrentState(IntersectionState newState);
        void getCurrentState(IntersectionState *currentState);

        std::unique_ptr<TrafficLight> north;
        std::unique_ptr<TrafficLight> south;
        std::unique_ptr<TrafficLight> west;
        std::unique_ptr<TrafficLight> east;

    private: 
        float _latitude;
        float _longitude;
        IntersectionState _currentState;

};


void Traffic_Task(void* p_arg);
void setupTrafficLights();

#endif