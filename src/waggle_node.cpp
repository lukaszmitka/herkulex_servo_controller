//
// Created by lukasz on 07.02.17.
//

#include "waggle_node.h"

int main(int argc, char **argv) {
   ros::init(argc, argv, "servo_waggle_node");
   ros::NodeHandle node_handle("~");

   node_handle.param<std::string>("topic_name", topic_name, DEFAULT_TPOIC_NAME);
   if (node_handle.getParam("topic_name", topic_name)) {
      std::cout << "Publishing to " << topic_name << " topic." << std::endl;
   } else {
      std::cout << "Publishing to default topic: " << topic_name << "." << std::endl;
   }

   ros::Publisher servo_publisher = node_handle
         .advertise<std_msgs::Float32>(topic_name, 1);
   servo_angle = 0;
   increase_angle = true;
   ros::Rate loop_rate(20);
   while (ros::ok()) {
      servo_goal.data = servo_angle;
      servo_publisher.publish(servo_goal);
      ros::spinOnce();
      loop_rate.sleep();
      if (increase_angle){
         servo_angle=servo_angle+0.1;
      } else {
         servo_angle = servo_angle-0.1;
      }
      if(servo_angle>90){
         increase_angle=false;
      }
      if(servo_angle<-90){
         increase_angle=true;
      }
   }

   return 0;
}
