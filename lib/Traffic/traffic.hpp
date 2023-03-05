#ifndef TRAFFIC_HG
#define TRAFFIC_HG
#include "../common.h"

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


void Traffic_Task(void* p_arg);
void setupTrafficLights();

#endif