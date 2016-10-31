//
// Created by lukasz on 30.09.16.
//

#ifndef HERKULEX_SERVO_DRIVER_COMPORTDRIVER_H
#define HERKULEX_SERVO_DRIVER_COMPORTDRIVER_H

#include <string>
#include <vector>
#include <iostream>
#include "rs232.h"

class ComPortDriver {
public:
    ComPortDriver(int port_number);

    ~ComPortDriver();

    std::vector<u_char> read_data();
    std::vector<u_char> read_data(bool wait_for_end_of_line);
    std::vector<u_char> read_data(int number_of_bytes);
    void flush_input();

    int send_data(std::vector<u_char> data);

    bool is_port_opened(){return port_opened;};
private:
    bool port_opened = false;
    int port_num;

    int init_comport(int port_number, long baud_rate);

    int close_comport(int port_number);

};


#endif //HERKULEX_SERVO_DRIVER_COMPORTDRIVER_H
