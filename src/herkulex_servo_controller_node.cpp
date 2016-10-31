//
// Created by lukasz on 24.10.16.
//

#include "herkulex_servo_controller_node.h"
#include <iostream>
#include <cstdlib>
#include <unistd.h>

HerkulexController *herkulexController;

void servo_control_callback(const std_msgs::Float32ConstPtr msg) {
   float degree = msg->data;
   uint16_t pos = (uint16_t) (degree + DRS0101_MIN_POS_OFFSET) / DRS0101_RESOLUTION;
   herkulexController->i_jog_control(253, pos);
}

int main(int argc, char **argv) {
   //int servo_id = 253; // DRS0101
   //int servo_id = 219; // DRS0602

   //std::cout << "Begin, init" << std::endl;
   ros::init(argc, argv, "herkulex_servo_controller_node");

   //std::cout << "Get node handle" << std::endl;
   ros::NodeHandle n("herkulex_servo_controller_node");

   ros::Subscriber sub = n.subscribe("servo_goal", 10, servo_control_callback);
   ros::Publisher orientation_pub = n.advertise<geometry_msgs::Quaternion>("servo_pos", 10);

   herkulexController = new HerkulexController(16);

   ros::Rate loop_rate(10);
   while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
   }
   //herkulexController->read_version();

   return 0;
}