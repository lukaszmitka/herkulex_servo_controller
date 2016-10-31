//
// Created by lukasz on 31.10.16.
//

#include "HerkulexController.h"


HerkulexController::HerkulexController(int port_number) {
   //std::cout << "Get ComPortDriver" << std::endl;
   comPortDriver = new ComPortDriver(port_number);
   if (comPortDriver->is_port_opened()) {
      comPort_opened = true;
      set_ack_policy(253, ACK_POLICY_REPLY_TO_ALL);
      set_min_position(253, 0);
      set_max_position(253, 1023);
      set_torque_control(253, TORQUE_CONTROL_TORQUE_ON);
      get_status_error(253);
   } else {
      comPort_opened = false;
      //std::cout << "Can not open ComPort" << std::endl;
   }
}

bool HerkulexController::has_acces_to_ComPort() {
   return comPort_opened;
}

HerkulexController::~HerkulexController() {
   comPortDriver->flush_input();
   comPortDriver->~ComPortDriver();
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

   std::cout << "Send data: ";
   for (int i = 0; i < packet.size(); i++) {
      std::cout << " " << (int) packet[i];
   }
   std::cout << std::endl;


   comPortDriver->flush_input();
   comPortDriver->send_data(packet);
   packet.clear();
   packet = comPortDriver->read_data(2);
   if (packet[0] == 0xFF && packet[1] == 0xFF) {
      tmp = comPortDriver->read_data(1);
      packet.insert(packet.end(), tmp.begin(), tmp.end());
      if (tmp[0] > 0) {
         tmp = comPortDriver->read_data(tmp[0] - 3);
         packet.insert(packet.end(), tmp.begin(), tmp.end());
      } else {
         std::cout << "Can not get proper response" << std::endl;
      }
   }
   std::cout << "Received data: ";
   for (int i = 0; i < packet.size(); i++) {
      std::cout << " " << (int) packet[i];
   }
   std::cout << std::endl;
   return packet;
}

void HerkulexController::ram_write(u_char servo_id, std::vector<u_char> data) {
   // RAM_WRITE command - 0x03
   std::vector<u_char> packet, tmp;
   packet = make_command_packet(servo_id, RAM_WRITE, data);

   std::cout << "Send data (ram_write): ";
   for (int i = 0; i < packet.size(); i++) {
      std::cout << " " << (int) packet[i];
   }
   std::cout << std::endl;


   comPortDriver->flush_input();
   comPortDriver->send_data(packet);
   packet.clear();
   packet = comPortDriver->read_data(2);
   if (packet[0] == 0xFF && packet[1] == 0xFF) {
      std::cout << "Header OK... ";
      tmp = comPortDriver->read_data(1);
      packet.insert(packet.end(), tmp.begin(), tmp.end());
      if (tmp[0] > 0) {

         tmp = comPortDriver->read_data(tmp[0] - 3);
         packet.insert(packet.end(), tmp.begin(), tmp.end());
      } else {
         std::cout << "Can not get proper response" << std::endl;
      }
   }
   std::cout << "Received data: ";
   for (int i = 0; i < packet.size(); i++) {
      std::cout << " " << (int) packet[i];
   }
   std::cout << std::endl;
   return;
}

void HerkulexController::set_ack_policy(u_char servo_id, u_char ack_policy) {
   std::cout << "Set ack policy" << std::endl;
   std::vector<u_char> data;
   data.push_back(ACK_POLICY_RAM_ADDR);
   data.push_back(0x01);
   data.push_back(ack_policy);
   ram_write(servo_id, data);
}

void HerkulexController::set_led(u_char servo_id, u_char led_state) {
   std::cout << "set led" << std::endl;
   std::vector<u_char> data;
   data.push_back(LED_CONTROL_RAM_ADDR);
   data.push_back(0x01);
   data.push_back(led_state);
   ram_write(servo_id, data);
   return;
}

u_char HerkulexController::get_led(u_char servo_id) {
   std::cout << "get led" << std::endl;
   std::vector<u_char> data;
   data.push_back(LED_CONTROL_RAM_ADDR);
   data.push_back(0x01);
   data = ram_read(servo_id, data);
   return 0;
}

void HerkulexController::set_min_position(u_char servo_id, u_int16_t min_pos) {
   std::cout << "set min" << std::endl;
   std::vector<u_char> data;
   data.push_back(MIN_POSITION_RAM_ADDR);
   data.push_back(0x02);
   data.push_back((u_char) (min_pos & 0xFF)); //LSB
   data.push_back((u_char) (min_pos >> 8)); // MSB
   ram_write(servo_id, data);
   return;
}

u_int16_t HerkulexController::get_min_position(u_char servo_id) {
   std::cout << "get min" << std::endl;
   std::vector<u_char> data;
   data.push_back(MIN_POSITION_RAM_ADDR);
   data.push_back(0x02);
   data = ram_read(servo_id, data);
   return 0;
}

void HerkulexController::set_max_position(u_char servo_id, u_int16_t max_pos) {
   std::cout << "set max" << std::endl;
   std::vector<u_char> data;
   data.push_back(MAX_POSITION_RAM_ADDR);
   data.push_back(0x02);
   data.push_back((u_char) (max_pos & 0xFF));
   data.push_back((u_char) (max_pos >> 8));
   ram_write(servo_id, data);
   return;
}

u_int16_t HerkulexController::get_max_position(u_char servo_id) {
   std::cout << "get max" << std::endl;
   std::vector<u_char> data;
   data.push_back(MAX_POSITION_RAM_ADDR);
   data.push_back(0x02);
   data = ram_read(servo_id, data);
   return 0;
}

void HerkulexController::set_torque_control(u_char servo_id, u_char torque_control_mode) {
   std::cout << "set torque control" << std::endl;
   std::vector<u_char> data;
   data.push_back(TORQUE_CONTROL_RAM_ADDR);
   data.push_back(0x01);
   data.push_back(torque_control_mode);
   ram_write(servo_id, data);
   return;
}

u_char HerkulexController::get_torque_control(u_char servo_id) {
   std::cout << "get torque control" << std::endl;
   std::vector<u_char> data;
   data.push_back(TORQUE_CONTROL_RAM_ADDR);
   data.push_back(0x01);
   data = ram_read(servo_id, data);
   return 0;
}

u_char HerkulexController::get_status_error(u_char servo_id) {
   std::cout << "get status error" << std::endl;
   std::vector<u_char> data;
   data.push_back(STATUS_ERROR_RAM_ADDR);
   data.push_back(0x01);
   data = ram_read(servo_id, data);
   return 0;
}

void HerkulexController::i_jog_control(u_char servo_id, u_int16_t position) {
   std::cout << "i jog control" << std::endl;
   // I_JOG command - 0x05
   std::vector<u_char> packet;
   std::vector<u_char> data, tmp;
   data.push_back((u_char) (position & 0xFF));
   data.push_back((u_char) (position >> 8));
   data.push_back(0x04);
   data.push_back(servo_id);
   data.push_back(0x3C);
   packet = make_command_packet(servo_id, I_JOG, data);

   std::cout << "Send data: ";
   for (int i = 0; i < packet.size(); i++) {
      std::cout << " " << (int) packet[i];
   }
   std::cout << std::endl;


   comPortDriver->flush_input();
   comPortDriver->send_data(packet);
   packet.clear();
   packet = comPortDriver->read_data(2);
   if (packet[0] == 0xFF && packet[1] == 0xFF) {
      std::cout << "Header OK... ";
      tmp = comPortDriver->read_data(1);
      packet.insert(packet.end(), tmp.begin(), tmp.end());
      if (tmp[0] > 0) {

         tmp = comPortDriver->read_data(tmp[0] - 3);
         packet.insert(packet.end(), tmp.begin(), tmp.end());
      } else {
         std::cout << "Can not get proper response" << std::endl;
      }
   }
   std::cout << "Received data: ";
   for (int i = 0; i < packet.size(); i++) {
      std::cout << " " << (int) packet[i];
   }
   std::cout << std::endl;
   return;
}

void HerkulexController::read_version() {
   return;
}
