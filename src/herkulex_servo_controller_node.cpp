//
// Created by lukasz on 24.10.16.
//

#include "herkulex_servo_controller_node.h"

HerkulexController *herkulexController;

uint16_t position_goal = 0;

void servo_control_callback(const std_msgs::Float32ConstPtr msg) {
   float degree = msg->data;
   //position_goal = (uint16_t) (degree + DRS0101_MIN_POS_OFFSET) / DRS0101_RESOLUTION;
   //position_goal = (degree + 90);
   position_goal = (degree / 0.02778) + 16384;
}

int main(int argc, char **argv) {
   //std::cout << "Begin, init" << std::endl;
   ros::init(argc, argv, "herkulex_servo_controller_node");

   //std::cout << "Get node handle" << std::endl;
   ros::NodeHandle node_handle("~");

   static tf::TransformBroadcaster tf_broadcaster;

   node_handle.param<bool>("publish_tf", publish_tf, false);
   if (node_handle.getParam("publish_tf", publish_tf)) {
      if (publish_tf) {
         std::cout << "tf will be published" << std::endl;
      } else {
         std::cout << "tf will be not published" << std::endl;
      }
   } else {
      std::cout << "tf will be not published" << std::endl;
   }

   node_handle.param<int>("servo_id", servo_id, DEFAULT_SERVO_ID);
   if (node_handle.getParam("servo_id", servo_id)) {
      std::cout << "Servo ID: " << servo_id << std::endl;
   } else {
      std::cout << "Use default servo ID: " << servo_id << std::endl;
   }

   node_handle.param<std::string>("servo_frame", servo_frame, DEFAULT_SERVO_FRAME);
   if (node_handle.getParam("servo_frame", servo_frame)) {
      std::cout << "Servo frame: " << servo_frame << std::endl;
   } else {
      std::cout << "Use default servo frame: " << servo_frame << std::endl;
   }

   node_handle.param<std::string>("servo_base_frame", servo_base_frame, DEFAULT_SERVO_BASE_FRAME);
   if (node_handle.getParam("servo_base_frame", servo_base_frame)) {
      std::cout << "Servo base frame: " << servo_base_frame << std::endl;
   } else {
      std::cout << "Use default servo base frame: " << servo_base_frame << std::endl;
   }

   node_handle.param<double>("z_offset", z_offset, DEFAULT_Z_OFFSET);
   if (node_handle.getParam("z_offset", z_offset)) {
      std::cout << "z axis offset: " << z_offset << std::endl;
   } else {
      std::cout << "Use default z axis offset: " << z_offset << std::endl;
   }

   node_handle.param<std::string>("position_goal", position_goal_topic, DEFAULT_POSITION_GOAL);
   if (node_handle.getParam("position_goal", position_goal_topic)) {
      std::cout << "Subscribing " << position_goal_topic << " topic." << std::endl;
   } else {
      std::cout << "Subscribing " << position_goal_topic << " topic." << std::endl;
   }

   node_handle.param<std::string>("servo_state", servo_state_topic, DEFAULT_STATE_TOPIC);
   if (node_handle.getParam("servo_state", servo_state_topic)) {
      std::cout << "Publishing to " << servo_state_topic << " topic." << std::endl;
   } else {
      std::cout << "Publishing to " << servo_state_topic << " topic." << std::endl;
   }


   ros::Subscriber position_subscriber = node_handle.subscribe(
         position_goal_topic, 1, servo_control_callback);
   ros::Publisher servo_state_publisher = node_handle
         .advertise<sensor_msgs::JointState>(servo_state_topic, 1);

   herkulexController = new HerkulexController("/dev/ttyUSB0", servo_id);

   ros::Rate loop_rate(50);
   while (ros::ok()) {
      u_int16_t abs_pos = herkulexController->get_absolute_position(servo_id);
      double current_pos = (((double) abs_pos) * DRS0101_RESOLUTION);
      sensor_msgs::JointState servo_state;

      servo_state.name.push_back(servo_base_frame);
      servo_state.position.push_back(current_pos);
      servo_state_publisher.publish(servo_state);

      if (publish_tf) {
         tf::Transform transform;
         transform.setOrigin(tf::Vector3(0.0, 0.0, z_offset));
         tf::Quaternion q;
         // switch from deg to rad
         q.setRPY(0, 0, current_pos);
         transform.setRotation(q);
         tf_broadcaster.sendTransform(
               tf::StampedTransform(transform, ros::Time::now(), servo_base_frame, servo_frame));
      }

      herkulexController->i_jog_control(servo_id, position_goal);
      ros::spinOnce();
      loop_rate.sleep();
   }

   return 0;
}