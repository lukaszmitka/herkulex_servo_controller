//
// Created by lukasz on 24.10.16.
//

#ifndef HERKULEX_SERVO_CONTROLLER_HERKULEX_SERVO_CONTROLLER_NODE_H
#define HERKULEX_SERVO_CONTROLLER_HERKULEX_SERVO_CONTROLLER_NODE_H

#include "HerkulexController.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/builtin_float.h"
#include "sensor_msgs/JointState.h"

#define DEFAULT_SERVO_ID 253
#define DEFAULT_SERVO_FRAME "herkulex"
#define DEFAULT_SERVO_BASE_FRAME "herkulex_base"
#define DEFAULT_Z_OFFSET 0
#define DEFAULT_POSITION_GOAL "herkulex_goal"
#define DEFAULT_STATE_TOPIC "servo_state"

bool publish_tf;
int servo_id; //int servo_id = 253; // DRS0101 // int servo_id = 219; // DRS0602
std::string servo_frame;
std::string servo_base_frame;
std::string position_goal_topic;
std::string servo_state_topic;
double z_offset;
#endif //HERKULEX_SERVO_CONTROLLER_HERKULEX_SERVO_CONTROLLER_NODE_H
