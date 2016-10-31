//
// Created by lukasz on 24.10.16.
//

#include "herkulex_servo_controller_node.h"
#include <iostream>
#include <cstdlib>
#include <unistd.h>

HerkulexController *herkulexController;

int main(int argc, char **argv) {
   //int servo_id = 253; // DRS0101
   //int servo_id = 219; // DRS0602
   int state;

   //std::cout << "Begin, init" << std::endl;
   ros::init(argc, argv, "herkulex_servo_controller_node");

   //std::cout << "Get node handle" << std::endl;
   ros::NodeHandle n("herkulex_servo_controller_node");


   herkulexController = new HerkulexController(16);
   herkulexController->read_version();

   return 0;
}