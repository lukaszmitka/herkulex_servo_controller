//
// Created by lukasz on 07.02.17.
//

#ifndef HERKULEX_SERVO_CONTROLLER_WAGGLE_NODE_H
#define HERKULEX_SERVO_CONTROLLER_WAGGLE_NODE_H

#define DEFAULT_TPOIC_NAME "servo_goal"

#include "ros/ros.h"
#include "std_msgs/builtin_float.h"

std::string topic_name;
double servo_angle;
std_msgs::Float32 servo_goal;
bool increase_angle;
#endif //HERKULEX_SERVO_CONTROLLER_WAGGLE_NODE_H
