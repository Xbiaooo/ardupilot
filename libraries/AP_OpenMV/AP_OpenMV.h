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

    float cx_flag;//表示x轴正负(1表示正，-1表示负)
    float cy_flag;//表示y轴正负

    // uint8_t cx_int;
    // uint8_t cx_dec;
    // uint8_t cy_int;
    // uint8_t cy_dec;
    // uint8_t cz_int;
    // uint8_t cz_dec;

    float cx, cy, cz;

    uint32_t last_frame_ms;

private:
    AP_HAL::UARTDriver *_port;              // UART used to send data to receiver

    uint8_t _step;

    uint8_t cx_int_temp;
    uint8_t cx_dec_temp;
    uint8_t cy_int_temp;
    uint8_t cy_dec_temp;
    uint8_t cz_int_temp;
    uint8_t cz_dec_temp;
};

