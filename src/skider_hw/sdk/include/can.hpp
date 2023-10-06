#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <chrono>
#include <iostream>

// #define CAN_NAME "can0"
#define YAW 0x205 //200+电调,204+电调
// #define PITCH 0x206
// #define COMMAND 0x1FF //200+电调,204+电调

#define CAN_NAME "can0"
#define WHEEL1 0x201
#define WHEEL2 0x202
#define WHEEL3 0x203
#define WHEEL4 0x204
#define CHASSIS 0x200


namespace transporter_sdk
{
    class Can
    {
    private:
        int socket_fd;
        struct sockaddr_can addr;
        struct ifreq interface_request;

    public:
        /**
         *  @brief  接收数据，根据id区分数据包，需要大于1000Hz频率接收
         *  @return error_code
         */
        int receive(uint &id, u_char *buf, u_char &dlc);

        /**
         * @brief   发送数据
         * @param   id  数据对应的ID
         * @param   buf 数据，长度小于等于8
         * @param   dlc 数据长度
         * @return  error_code
         */
        int send(uint id, u_char *buf, u_char dlc);

        Can();
        ~Can();

        enum CanError
        {
            SUCCESS,
            DLC_ERROR,
            WRITE_ERROR,
            READ_ERROR,
            TIME_ERROR
        };
    };

} // namespace transporter_sdk
