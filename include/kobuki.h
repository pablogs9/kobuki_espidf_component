#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdbool.h>

#if !defined(__BYTE_ORDER__)
#error ENDIANESS NOT DEFINED: Required for Kobuki library
#endif

// #define KOBUKI_DEBUG

// **********************************
// *        KOBUKI SUBPAYLOADS      *
// **********************************

typedef uint8_t kobuki_subpayload_type_t;

// TODO(pablogs): Check when this is necessary
static inline void * kobuki_swap_endianness( uint8_t * buff, uint8_t size){
    if (size == 16){ // swap 0-1
        buff[0] ^= buff[1]; buff[1] ^= buff[0]; buff[0] ^= buff[1];
    } else if (size == 32) { // swap 0-3 and 1-2
        buff[0] ^= buff[3]; buff[3] ^= buff[0]; buff[0] ^= buff[3];
        buff[1] ^= buff[2]; buff[2] ^= buff[1]; buff[1] ^= buff[2];
    }
    return (void *) buff;
}

// ####### BASIC SENSOR DATA #######

// ID and length
#define KOBUKI_BASIC_SENSOR_DATA 1
#define KOBUKI_BASIC_SENSOR_DATA_LEN 15

// Relevant types
#define KOBUKI_BASIC_SENSOR_DATA_BUMPER_RIGHT 0x1
#define KOBUKI_BASIC_SENSOR_DATA_BUMPER_CENTRAL 0x2
#define KOBUKI_BASIC_SENSOR_DATA_BUMPER_LEFT 0x4
#define KOBUKI_BASIC_SENSOR_DATA_WHEEL_DROP_RIGHT 0x1
#define KOBUKI_BASIC_SENSOR_DATA_WHEEL_DROP_LEFT 0x2
#define KOBUKI_BASIC_SENSOR_DATA_CLIFF_RIGHT 0x1
#define KOBUKI_BASIC_SENSOR_DATA_CLIFF_CENTER 0x2
#define KOBUKI_BASIC_SENSOR_DATA_CLIFF_LEFT 0x4
#define KOBUKI_BASIC_SENSOR_DATA_BUTTON_0 0x1
#define KOBUKI_BASIC_SENSOR_DATA_BUTTON_1 0x2
#define KOBUKI_BASIC_SENSOR_DATA_BUTTON_2 0x4
#define KOBUKI_BASIC_SENSOR_DATA_CHARGER_DISCHARGED 0
#define KOBUKI_BASIC_SENSOR_DATA_CHARGER_DOCKING_CHARGED 2
#define KOBUKI_BASIC_SENSOR_DATA_CHARGER_DOCKING_CHARGING 6
#define KOBUKI_BASIC_SENSOR_DATA_CHARGER_ADAPTER_CHARGED 18
#define KOBUKI_BASIC_SENSOR_DATA_CHARGER_ADAPTER_CHARGING 22
#define KOBUKI_BASIC_SENSOR_DATA_OVERCURRENT_LEFT 0x1
#define KOBUKI_BASIC_SENSOR_DATA_OVERCURRENT_RIGHT 0x2

// Elements
#define ELEMENTS_NAME \
            X(timestamp, 16) \
            X(bumper, 8) \
            X(wheel_drop, 8) \
            X(cliff, 8) \
            X(left_encoder, 16) \
            X(right_encoder, 16) \
            X(left_pwm, 8) \
            X(right_pwm, 8) \
            X(button, 8) \
            X(charger, 8) \
            X(battery, 8) \
            X(overcurrent, 8) \

#define NAME kobuki_basic_sensor_data
#include "generate_support.def"
#undef ELEMENTS_NAME
#undef NAME

// ####### DOCKING IR #######

// ID and length
#define KOBUKI_DOCKING_IR 3
#define KOBUKI_DOCKING_IR_LEN 3

// Relevant types
#define KOBUKI_DOCKING_IR_NEAR_LEFT 0x1
#define KOBUKI_DOCKING_IR_NEAR_CENTER 0x2
#define KOBUKI_DOCKING_IR_NEAR_RIGHT 0x4
#define KOBUKI_DOCKING_IR_FAR_CENTER 0x8
#define KOBUKI_DOCKING_IR_FAR_LEFT 0x10
#define KOBUKI_DOCKING_IR_FAR_RIGHT 0x20


// Elements
#define ELEMENTS_NAME \
            X(right_signal, 8) \
            X(central_signal, 8) \
            X(left_signal, 8) \

#define NAME kobuki_docking_ir
#include "generate_support.def"
#undef ELEMENTS_NAME
#undef NAME

// ####### INERTIAL SENSOR DATA #######

// ID and length
#define KOBUKI_INERTIAL_SENSOR_DATA 4
#define KOBUKI_INERTIAL_SENSOR_DATA_LEN 7

// Elements
#define ELEMENTS_NAME \
            X(angle, 16) \
            X(angle_rate, 16) \
            X(unused_1, 8) \
            X(unused_2, 8) \
            X(unused_3, 8) \

#define NAME kobuki_inertial_sensor_data
#include "generate_support.def"
#undef ELEMENTS_NAME
#undef NAME

// ####### CLIFF SENSOR DATA #######

// ID and length
#define KOBUKI_CLIFF_SENSOR_DATA 5
#define KOBUKI_CLIFF_SENSOR_DATA_LEN 6

// Elements
#define ELEMENTS_NAME \
            X(right, 16) \
            X(center, 16) \
            X(left, 16) \

#define NAME kobuki_cliff_sensor_data
#include "generate_support.def"
#undef ELEMENTS_NAME
#undef NAME

// ####### CURRENT DATA #######

// ID and length
#define KOBUKI_CURRENT_DATA 6
#define KOBUKI_CURRENT_DATA_LEN 2

// Elements
#define ELEMENTS_NAME \
            X(left_motor, 8) \
            X(right_motor, 8) \

#define NAME kobuki_current_data
#include "generate_support.def"
#undef ELEMENTS_NAME
#undef NAME

// ####### HARDWARE VERSION #######

// ID and length
#define KOBUKI_HARDWARE_VERSION 10
#define KOBUKI_HARDWARE_VERSION_LEN 4

// Elements
#define ELEMENTS_NAME \
            X(patch, 8) \
            X(minor, 8) \
            X(mayor, 8) \
            X(unused, 8) \

#define NAME kobuki_hardware_version
#include "generate_support.def"
#undef ELEMENTS_NAME
#undef NAME

// ####### FIRMWARE VERSION #######

// ID and length
#define KOBUKI_FIRMWARE_VERSION 11
#define KOBUKI_FIRMWARE_VERSION_LEN 4

// Elements
#define ELEMENTS_NAME \
            X(patch, 8) \
            X(minor, 8) \
            X(mayor, 8) \
            X(unused, 8) \

#define NAME kobuki_firmware_version
#include "generate_support.def"
#undef ELEMENTS_NAME
#undef NAME

// ####### RAW DATA 3D GYRO #######

// ID and length
#define KOBUKI_RAW_DATA_3D_GYRO 13

// VARIABLE LEN: NOT CONSIDERED

// ####### GPIO DATA #######

// ID and length
#define KOBUKI_GPIO_DATA 16
#define KOBUKI_GPIO_DATA_LEN 16

// Elements
#define ELEMENTS_NAME \
            X(digital_input, 16) \
            X(analog_ch0, 16) \
            X(analog_ch1, 16) \
            X(analog_ch2, 16) \
            X(analog_ch3, 16) \
            X(unused_1, 16) \
            X(unused_2, 16) \
            X(unused_3, 16) \


#define NAME kobuki_gpio_data
#include "generate_support.def"
#undef ELEMENTS_NAME
#undef NAME

// ####### UDID DATA #######

// ID and length
#define KOBUKI_UDID_DATA 19
#define KOBUKI_UDID_DATA_LEN 16

// Elements
#define ELEMENTS_NAME \
            X(UDID_0, 32) \
            X(UDID_1, 32) \
            X(UDID_2, 32) \

#define NAME kobuki_udid_data
#include "generate_support.def"
#undef ELEMENTS_NAME
#undef NAME

// ####### CONTROLLER INFO #######

// ID and length
#define KOBUKI_CONTROLLER_INFO 21
#define KOBUKI_CONTROLLER_INFO_LEN 21

// Elements
#define ELEMENTS_NAME \
            X(type, 8) \
            X(P_gain, 32) \
            X(I_gain, 32) \
            X(D_gain, 32) \

#define NAME kobuki_controller_info
#include "generate_support.def"
#undef ELEMENTS_NAME
#undef NAME

// **********************************
// *         KOBUKI TYPES           *
// **********************************

typedef struct {
    float x;
    float y;
    float theta;
    float linear_velocity;
    float angular_velocity;
    bool init;
} kobuki_odometry_t;

typedef struct {
    kobuki_udid_data_t hw_id;
    kobuki_odometry_t odometry;
    kobuki_basic_sensor_data_t last_basic_sensor_data;
} kobuki_status_t;

typedef struct {
    kobuki_subpayload_type_t type;
    union {
        kobuki_basic_sensor_data_t basic_sensor_data;
        kobuki_docking_ir_t docking_ir;
        kobuki_inertial_sensor_data_t inertial_sensor_data;
        kobuki_cliff_sensor_data_t cliff_sensor_data;
        kobuki_current_data_t current_data;
        kobuki_hardware_version_t hardware_version;
        kobuki_firmware_version_t firmware_version;
        kobuki_gpio_data_t gpio_data;
        kobuki_udid_data_t udid_data;
        kobuki_controller_info_t controller_info;
    };
    
} kobuki_subpayload_t;

typedef void (*kobuki_subpayload_callback_t)(kobuki_subpayload_t *);

// **********************************
// *           KOBUKI API           *
// **********************************

void kobuki_init_serial();
void kobuki_set_subpayload_callback(kobuki_subpayload_callback_t cb);
void kobuki_set_emergency_callback(kobuki_subpayload_callback_t cb);
void kobuki_set_speed_command(float translation, float rotation);
void kobuki_set_sound_command(float frequency, uint8_t duration_ms);
kobuki_status_t kobuki_get_status();
void kobuki_loop();