/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>


class AP_OpenMV {
public:
    AP_OpenMV();

    /* Do not allow copies */
    AP_OpenMV(const AP_OpenMV &other) = delete;
    AP_OpenMV &operator=(const AP_OpenMV&) = delete;

    // init - perform require initialisation including detecting which protocol to use
    void init(const AP_SerialManager& serial_manager);

    // update flight control mode. The control mode is vehicle type specific
    bool update(void);

    //目标中心点在图像坐标系中的坐标，及目标距无人机当前的距离
    uint8_t cx;
    uint8_t cy;
    //uint8_t distance;

    //openmv焦距
    float focal_length = 2.8e-3f;
    //openmv一个像素点的宽度（openmv视角：HFOV=70.8° VFOV=55.6°）
    float px_length = focal_length * (tanf(35.4f)/80 + tanf(27.8f)/60)/2;

    uint32_t last_frame_ms;

private:
    AP_HAL::UARTDriver *_port;              // UART used to send data to receiver

    uint8_t _step;

    //暂存串口发送过来的cx、cy和distance数据
    uint8_t _cx_temp;
    uint8_t _cy_temp;
    uint8_t _distance_temp;
};


