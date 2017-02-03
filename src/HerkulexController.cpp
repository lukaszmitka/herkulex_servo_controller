//
// Created by lukasz on 31.10.16.
//

#include "HerkulexController.h"

HerkulexController::HerkulexController(std::string port, int servo_id, uint8_t servo_type,
                                       uint16_t timeout) {
   internal_servo_id = servo_id;
   switch (servo_type) {
      case SERVO_TYPE_DRS_0101:
         internal_servo_type = servo_type;
         internal_max_pos = DRS_0101_MAX_POS;
         internal_min_pos = DRS_0101_MIN_POS;
         internal_res = DRS_0101_RESOLUTION;
         internal_zero = DRS_0101_ZERO_POS;
         break;
      case SERVO_TYPE_DRS_0602:
         internal_servo_type = servo_type;
         internal_max_pos = DRS_0602_MAX_POS;
         internal_min_pos = DRS_0602_MIN_POS;
         internal_res = DRS_0602_RESOLUTION;
         internal_zero = DRS_0602_ZERO_POS;
         break;
      default:
         internal_servo_type = SERVO_TYPE_UNKNOWN;
   }

   if (internal_servo_type != SERVO_TYPE_UNKNOWN) {
      //std::cout << "Create Serial instance" << std::endl;
      my_serial = new serial::Serial(port, 115200, serial::Timeout::simpleTimeout(timeout));

      //std::cout << "Check if port is opened" << std::endl;
      if (my_serial->isOpen()) {
         comPort_opened = true;
         set_ack_policy(internal_servo_id, ACK_POLICY_REPLY_TO_ALL);
         set_min_position(internal_servo_id, internal_min_pos);
         set_max_position(internal_servo_id, internal_max_pos);
         set_torque_control(internal_servo_id, TORQUE_CONTROL_TORQUE_ON);
         if (get_status_error(internal_servo_id) != 0) {
            servo_detected = false;
            return;
         } else {
            servo_detected = true;
         }
         set_led(internal_servo_id, LED_CONTROL_BLUE);
      } else {
         comPort_opened = false;
         servo_detected = false;
         std::cout << "Can not open serial port" << std::endl;
      }
   } else {
      servo_detected = false;
   }
}

bool HerkulexController::has_acces_to_ComPort() {
   return comPort_opened;
}

HerkulexController::~HerkulexController() {
   my_serial->flush();
   my_serial->close();
}

void HerkulexController::calculate_checksum(u_char *checksum_1, u_char *checksum_2,
                                            u_char *packet_size,
                                            u_char servo_id, u_char command,
                                            std::vector<u_char> data) {
   u_char checksum_tmp = 0;

   *packet_size = (u_char) 7 + (u_char) data.size();
   checksum_tmp = *packet_size ^ servo_id ^ command;
   for (int i = 0; i < data.size(); i++) {
      checksum_tmp = checksum_tmp ^ data[i];
   }
   *checksum_1 = (checksum_tmp) & (u_char) 0xFE;
   *checksum_2 = (~*checksum_1) & (u_char) 0xFE;
   //std::cout << "Checksum 1: " << (int) 0x30 << ", calculated: " << (int) checksum_1 << std::endl;
   //std::cout << "Checksum 2: " << (int) 0xCE << ", calculated: " << (int) checksum_2 << std::endl;
   return;
}

std::vector<u_char> HerkulexController::make_command_packet(u_char servo_id, u_char command,
                                                            std::vector<u_char> data) {
   std::vector<u_char> command_packet;

   u_char checksum_1;
   u_char checksum_2;
   u_char packet_size;
   calculate_checksum(&checksum_1, &checksum_2, &packet_size, servo_id, command, data);

   command_packet.push_back(0xFF);
   command_packet.push_back(0xFF);
   command_packet.push_back(packet_size);
   command_packet.push_back(servo_id);
   command_packet.push_back(command);
   command_packet.push_back(checksum_1);
   command_packet.push_back(checksum_2);

   for (int i = 0; i < data.size(); i++) {
      command_packet.push_back(data[i]);
   }
   return command_packet;
}

std::vector<u_char> HerkulexController::ram_read(u_char servo_id, std::vector<u_char> data) {
   std::vector<u_char> packet, tmp;
   packet = make_command_packet(servo_id, RAM_READ, data);
   /*std::cout << "Send data (RAM_READ): ";
   for (int i = 0; i < packet.size(); i++) {
      std::cout << " " << (int) packet[i];
   }
   std::cout << std::endl;*/
   my_serial->flushInput();
   my_serial->write(packet);
   packet.clear();
   my_serial->read(packet, 2);
   if (packet.size() < 2) {
      std::cout << "Not enough data received" << std::endl;
   }
   else {
      if (packet[0] == 0xFF && packet[1] == 0xFF) {
         my_serial->read(tmp, 1);
         response_len = tmp[0];

         if (response_len > 0 && response_len < 15) {
            packet.insert(packet.end(), tmp.begin(), tmp.end());
            tmp.clear();
            my_serial->read(tmp, response_len - 3);
            packet.insert(packet.end(), tmp.begin(), tmp.end());
         } else {
            std::cout << "Can not get proper response" << std::endl;
         }
      }
      /*std::cout << "Received data: ";
      for (int i = 0; i < packet.size(); i++) {
         std::cout << " " << (int) packet[i];
      }
      std::cout << std::endl;*/
   }
   return packet;
}

void HerkulexController::ram_write(u_char servo_id, std::vector<u_char> data) {
   // RAM_WRITE command - 0x03
   std::vector<u_char> packet, tmp;
   packet = make_command_packet(servo_id, RAM_WRITE, data);
   /*std::cout << "Send data (ram_write): ";
   for (int i = 0; i < packet.size(); i++) {
      std::cout << " " << (int) packet[i];
   }
   std::cout << std::endl;*/
   my_serial->flushInput();
   my_serial->write(packet);
   packet.clear();
   my_serial->read(packet, 2);
   if (packet.size() < 2) {
      std::cout << "Not enugh data received" << std::endl;
   } else {
      if (packet[0] == 0xFF && packet[1] == 0xFF) {
         //std::cout << "Header OK... ";
         my_serial->read(tmp, 1);
         response_len = tmp[0];
         if (response_len > 0 && response_len < 15) {
            packet.insert(packet.end(), tmp.begin(), tmp.end());
            tmp.clear();
            my_serial->read(tmp, response_len - 3);
            packet.insert(packet.end(), tmp.begin(), tmp.end());
         } else {
            std::cout << "Can not get proper response" << std::endl;
         }
      }
      /*std::cout << "Received data: ";
      for (int i = 0; i < packet.size(); i++) {
         std::cout << " " << (int) packet[i];
      }
      std::cout << std::endl;*/
   }
   return;
}

void HerkulexController::set_ack_policy(u_char servo_id, u_char ack_policy) {
   //std::cout << "Set ack policy" << std::endl;
   std::vector<u_char> data;
   data.push_back(ACK_POLICY_RAM_ADDR);
   data.push_back(0x01);
   data.push_back(ack_policy);
   ram_write(servo_id, data);
}

void HerkulexController::set_led(u_char servo_id, u_char led_state) {
   //std::cout << "set led" << std::endl;
   std::vector<u_char> data;
   data.push_back(LED_CONTROL_RAM_ADDR);
   data.push_back(0x01);
   data.push_back(led_state);
   ram_write(servo_id, data);
   return;
}

u_char HerkulexController::get_led(u_char servo_id) {
   //std::cout << "get led" << std::endl;
   std::vector<u_char> data;
   data.push_back(LED_CONTROL_RAM_ADDR);
   data.push_back(0x01);
   data = ram_read(servo_id, data);
   return 0;
}

void HerkulexController::set_min_position(u_char servo_id, u_int16_t min_pos) {
   //std::cout << "set min" << std::endl;
   std::vector<u_char> data;
   data.push_back(MIN_POSITION_RAM_ADDR);
   data.push_back(0x02);
   data.push_back((u_char) (min_pos & 0xFF)); //LSB
   data.push_back((u_char) (min_pos >> 8)); // MSB
   ram_write(servo_id, data);
   return;
}

u_int16_t HerkulexController::get_min_position(u_char servo_id) {
   //std::cout << "get min" << std::endl;
   std::vector<u_char> data;
   data.push_back(MIN_POSITION_RAM_ADDR);
   data.push_back(0x02);
   data = ram_read(servo_id, data);
   return 0;
}

void HerkulexController::set_max_position(u_char servo_id, u_int16_t max_pos) {
   //std::cout << "set max" << std::endl;
   std::vector<u_char> data;
   data.push_back(MAX_POSITION_RAM_ADDR);
   data.push_back(0x02);
   data.push_back((u_char) (max_pos & 0xFF));
   data.push_back((u_char) (max_pos >> 8));
   ram_write(servo_id, data);
   return;
}

u_int16_t HerkulexController::get_max_position(u_char servo_id) {
   //std::cout << "get max" << std::endl;
   std::vector<u_char> data;
   data.push_back(MAX_POSITION_RAM_ADDR);
   data.push_back(0x02);
   data = ram_read(servo_id, data);
   return 0;
}

void HerkulexController::set_torque_control(u_char servo_id, u_char torque_control_mode) {
   //std::cout << "set torque control" << std::endl;
   std::vector<u_char> data;
   data.push_back(TORQUE_CONTROL_RAM_ADDR);
   data.push_back(0x01);
   data.push_back(torque_control_mode);
   ram_write(servo_id, data);
   return;
}

u_char HerkulexController::get_torque_control(u_char servo_id) {
   //std::cout << "get torque control" << std::endl;
   std::vector<u_char> data;
   data.push_back(TORQUE_CONTROL_RAM_ADDR);
   data.push_back(0x01);
   data = ram_read(servo_id, data);
   return 0;
}

u_char HerkulexController::get_status_error(u_char servo_id) {
   //std::cout << "get status error" << std::endl;
   std::vector<u_char> data;
   data.push_back(STATUS_ERROR_RAM_ADDR);
   data.push_back(0x01);
   data = ram_read(servo_id, data);
   return 0;
}

void HerkulexController::i_jog_control(u_char servo_id, u_int16_t position) {
   //std::cout << "i jog control" << std::endl;
   // I_JOG command - 0x05
   std::vector<u_char> packet;
   std::vector<u_char> data, tmp;
   data.push_back((u_char) (position & 0xFF));
   data.push_back((u_char) (position >> 8));
   data.push_back(0x08);
   data.push_back(servo_id);
   data.push_back(0x00);
   packet = make_command_packet(servo_id, I_JOG, data);

   /*std::cout << "Send data: ";
   for (int i = 0; i < packet.size(); i++) {
      std::cout << " " << (int) packet[i];
   }
   std::cout << std::endl;*/

   my_serial->flushInput();
   my_serial->write(packet);
   packet.clear();
   my_serial->read(packet, 2);
   if (packet[0] == 0xFF && packet[1] == 0xFF) {
      //std::cout << "Header OK... ";
      my_serial->read(tmp, 1);
      response_len = tmp[0];

      if (response_len > 0 && response_len < 15) {
         packet.insert(packet.end(), tmp.begin(), tmp.end());
         tmp.clear();
         my_serial->read(tmp, response_len - 3);
         packet.insert(packet.end(), tmp.begin(), tmp.end());
      } else {
         std::cout << "Can not get proper response" << std::endl;
      }
   }
   /*std::cout << "Received data: ";
   for (int i = 0; i < packet.size(); i++) {
      std::cout << " " << (int) packet[i];
   }
   std::cout << std::endl;*/
   return;
}

void HerkulexController::i_jog_control(u_char servo_id, double position) {
   uint16_t goal_pos = (position / internal_res) + internal_zero;
   i_jog_control(servo_id, goal_pos);
   return;
}

u_int16_t HerkulexController::get_absolute_position(u_char servo_id) {
   uint16_t abs_pos = 0;
   //std::cout << "get absolute position" << std::endl;
   std::vector<u_char> data;
   data.push_back(ABSOLUTE_POSITION_RAM_ADDR);
   data.push_back(0x02);
   data = ram_read(servo_id, data);

   /*std::cout << "Received data: ";
   for (int i = 0; i < data.size(); i++) {
      std::cout << " " << (int) data[i];
   }*/


   u_char hi_byte = data[10];
   u_char low_byte = data[9];
   abs_pos = (hi_byte << 8) + low_byte;

   //std::cout << " Abs pos" << abs_pos;
   //std::cout << std::endl;

   return abs_pos;
}

double HerkulexController::get_absolute_position_deg(u_char servo_id) {
   uint16_t abs_pos = get_absolute_position(servo_id);
   double deg_pos = (((double) abs_pos) - internal_zero) * internal_res;
   //std::cout << "Abs pos" << abs_pos << ", deg: " << deg_pos << std::endl;
   return deg_pos;
}

double HerkulexController::get_absolute_position_rad(u_char servo_id) {
   uint16_t abs_pos = get_absolute_position(servo_id);
   double rad_pos = (abs_pos - internal_zero) * internal_res * M_PI / 180;
   return rad_pos;
}

void HerkulexController::read_version() {
   return;
}
