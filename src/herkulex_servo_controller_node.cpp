//
// Created by lukasz on 24.10.16.
//

#include "herkulex_servo_controller_node.h"

HerkulexController *herkulexController;

uint16_t position_goal = 0;

void servo_control_callback(const std_msgs::Float32ConstPtr msg) {
   float degree = msg->data;
   position_goal = (uint16_t) (degree + DRS0101_MIN_POS_OFFSET) / DRS0101_RESOLUTION;
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
      geometry_msgs::Quaternion quaternion;
      tf::Vector3 vector(0, 0, 1);
      u_int16_t abs_pos = herkulexController->get_absolute_position(253);
      double current_pos = (((double) abs_pos) * DRS0101_RESOLUTION) - DRS0101_MIN_POS_OFFSET;
      tf::Quaternion quat(vector, current_pos);
      quaternion.x = quat.x();
      quaternion.y = quat.y();
      quaternion.z = quat.z();
      quaternion.w = quat.w();
      orientation_pub.publish(quaternion);

      herkulexController->i_jog_control(253, position_goal);
      ros::spinOnce();
      loop_rate.sleep();
   }

   return 0;
}