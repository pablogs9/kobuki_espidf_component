#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>

#include <kobuki.h>
#include "kobuki_platform.h"

// **********************************
// *     KOBUKI CONFIGURATION       *
// **********************************

#ifndef KOBUKI_UART_READ_WAIT_MS
#define KOBUKI_UART_READ_WAIT_MS 1
#endif

#ifndef KOBUKI_RATE_MS
#define KOBUKI_RATE_MS 20
#endif

#ifndef KOBUKI_PROCESSING_BUFFER
#define KOBUKI_PROCESSING_BUFFER 2048
#endif

// **********************************
// *       KOBUKI CONSTANTS         *
// **********************************

#define KOBUKI_WHEELBASE 0.230 // meters
#define KOBUKI_TICKS_PER_METER 1./11724.41658029856624751591


// **********************************
// *       KOBUKI PORT LAYER        *
// **********************************

extern void kobuki_init_serial();
extern void kobuki_write_bytes(uint8_t * buf, size_t len);
extern size_t kobuki_read_bytes(uint8_t * buf, size_t len);

// **********************************
// *         KOBUKI COMMANDS        *
// **********************************

#define KOBUKI_COMMAND_HEADER_LEN 2

#define KOBUKI_BASE_CONTROL_COMMAND 1
#define KOBUKI_BASE_CONTROL_COMMAND_LEN 4

#define KOBUKI_SOUND_COMMAND 1
#define KOBUKI_SOUND_COMMAND_LEN 3

// **********************************
// *      KOBUKI STATE MACHINE      *
// **********************************

typedef enum {
    NO_SYNC = 0,
    HEADER_0,
    HEADER_1,
    PACKET_LEN,
    READING_PAYLOAD,
    CHECK_CRC,
    CRC_ERROR,
    PROCESING,
    KOBUKI_UART_SM_STATES,
} kobuki_uart_state_t;

typedef struct {
    kobuki_uart_state_t state;
    uint8_t buffer[KOBUKI_PROCESSING_BUFFER];
    uint8_t write_pointer;
    uint8_t read_pointer;
    uint8_t payload_size;
} kobuki_state_machine_t;

typedef struct {
    kobuki_state_machine_t state_machine;
    kobuki_subpayload_callback_t subpayload_callback;
    kobuki_subpayload_callback_t emergency_callback;
    kobuki_status_t robot_status;
    uint8_t base_control_command[KOBUKI_BASE_CONTROL_COMMAND_LEN + KOBUKI_COMMAND_HEADER_LEN];
    uint8_t sound_command[KOBUKI_SOUND_COMMAND_LEN + KOBUKI_COMMAND_HEADER_LEN];
} kobuki_robot_t;

static kobuki_robot_t robot = {0};
pthread_mutex_t robot_lock;

void kobuki_crc_error_hook(){
#ifdef KOBUKI_DEBUG
    printf("Kobuki CRC error\n");
#endif
}

void kobuki_subpayload_error_hook(){
#ifdef KOBUKI_DEBUG
    printf("Kobuki subpayload error\n");
#endif
}

bool kobuki_check_crc(){
    uint8_t cs = 0;
    cs ^= robot.state_machine.payload_size;
    for (size_t i = 0; i < robot.state_machine.write_pointer; i++){
        cs ^= robot.state_machine.buffer[i];
    }
    robot.state_machine.payload_size--; // Remove CRC from the data
    return cs ? false : true;
}

kobuki_status_t kobuki_get_status(){
    return robot.robot_status;
}

void kobuki_update_odometry(kobuki_basic_sensor_data_t * msg){
    if (!robot.robot_status.odometry.init){
        robot.robot_status.odometry.x = 0.0;
        robot.robot_status.odometry.y = 0.0;
        robot.robot_status.odometry.theta = 0.0;
        robot.robot_status.odometry.init = true;
    }else{
        double elapsed_time_ms = (msg->timestamp > robot.robot_status.last_basic_sensor_data.timestamp)  ? 
                                    msg->timestamp - robot.robot_status.last_basic_sensor_data.timestamp :
                                    msg->timestamp + ((uint16_t)0xFFFF - robot.robot_status.last_basic_sensor_data.timestamp);

        double dl = KOBUKI_TICKS_PER_METER * (msg->left_encoder - robot.robot_status.last_basic_sensor_data.left_encoder);
        double dr = KOBUKI_TICKS_PER_METER * (msg->right_encoder  - robot.robot_status.last_basic_sensor_data.right_encoder);
        double dx = 0, dy = 0, dtheta = (dr-dl)/KOBUKI_WHEELBASE;

        if (dl!=dr) {
            double R = 0.5 * (dr + dl) / dtheta;
            dx = R * sin(dtheta);
            dy = R * (1 - cos(dtheta));
        } else {
            dx = dr;
        }

        double s = sin(robot.robot_status.odometry.theta), c = cos(robot.robot_status.odometry.theta);
        double diff_x = c * dx - s * dy;

        robot.robot_status.odometry.linear_velocity = diff_x / (elapsed_time_ms / 1000.0);
        robot.robot_status.odometry.x += diff_x;
        robot.robot_status.odometry.y += s * dx + c * dy;
        robot.robot_status.odometry.theta += dtheta;
        robot.robot_status.odometry.theta = fmod(robot.robot_status.odometry.theta + 4*M_PI, 2*M_PI);
        
        if (robot.robot_status.odometry.theta > M_PI){
            robot.robot_status.odometry.theta -= 2*M_PI;
        }
    }
}

bool kobuki_check_emergency_status(kobuki_basic_sensor_data_t * msg){
    return  (  msg->bumper 
            || msg->wheel_drop 
            || msg->cliff 
            || msg->overcurrent);
}

void kobuki_process_payload(){
    while (robot.state_machine.read_pointer < robot.state_machine.payload_size ){
        kobuki_subpayload_t submessage;

        submessage.type = robot.state_machine.buffer[robot.state_machine.read_pointer++];
        uint8_t submessage_len = robot.state_machine.buffer[robot.state_machine.read_pointer++];
        bool known_type = true;
        switch (submessage.type){
            case KOBUKI_BASIC_SENSOR_DATA:
                decode_kobuki_basic_sensor_data(&robot.state_machine.buffer[robot.state_machine.read_pointer], &submessage.basic_sensor_data);
                if (kobuki_check_emergency_status(&submessage.basic_sensor_data)){
                    uint8_t emergency_stop_command[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x00}; // Sets all velocities to zero
                    kobuki_write_bytes(emergency_stop_command, sizeof(emergency_stop_command));
                    if (robot.emergency_callback) { robot.emergency_callback(&submessage); }
                }
                kobuki_update_odometry(&submessage.basic_sensor_data);
                robot.robot_status.last_basic_sensor_data = submessage.basic_sensor_data;
                break;
            case KOBUKI_DOCKING_IR:
                decode_kobuki_docking_ir(&robot.state_machine.buffer[robot.state_machine.read_pointer], &submessage.docking_ir);
                break;
            case KOBUKI_INERTIAL_SENSOR_DATA:
                decode_kobuki_inertial_sensor_data(&robot.state_machine.buffer[robot.state_machine.read_pointer], &submessage.inertial_sensor_data);
                break;
            case KOBUKI_CLIFF_SENSOR_DATA:
                decode_kobuki_cliff_sensor_data(&robot.state_machine.buffer[robot.state_machine.read_pointer], &submessage.cliff_sensor_data);
                break;
            case KOBUKI_CURRENT_DATA:
                decode_kobuki_current_data(&robot.state_machine.buffer[robot.state_machine.read_pointer], &submessage.current_data);
                break;
            case KOBUKI_HARDWARE_VERSION:
                decode_kobuki_hardware_version(&robot.state_machine.buffer[robot.state_machine.read_pointer], &submessage.hardware_version);
                break;
            case KOBUKI_FIRMWARE_VERSION:
                decode_kobuki_firmware_version(&robot.state_machine.buffer[robot.state_machine.read_pointer], &submessage.firmware_version);
                break;
            case KOBUKI_RAW_DATA_3D_GYRO:
                // Not implemented
                break;
            case KOBUKI_GPIO_DATA:
                decode_kobuki_gpio_data(&robot.state_machine.buffer[robot.state_machine.read_pointer], &submessage.gpio_data);
                break;
            case KOBUKI_UDID_DATA:
                decode_kobuki_udid_data(&robot.state_machine.buffer[robot.state_machine.read_pointer], &submessage.udid_data);
                robot.robot_status.hw_id = submessage.udid_data;
                break;
            case KOBUKI_CONTROLLER_INFO:
                decode_kobuki_controller_info(&robot.state_machine.buffer[robot.state_machine.read_pointer], &submessage.controller_info);
                break;
            default:
                known_type = false;
                break;
        }
        
        if (robot.subpayload_callback != NULL && known_type){
            robot.subpayload_callback(&submessage);
        }
        
        robot.state_machine.read_pointer += submessage_len;
    }
}

void kobuki_state_machine(uint8_t input){
    switch (robot.state_machine.state){
    case NO_SYNC:
        robot.state_machine.state = (input == 0xAA) ? HEADER_0 : NO_SYNC; break;
    case HEADER_0:
        robot.state_machine.state = (input == 0x55) ? HEADER_1 : NO_SYNC; break;
    case HEADER_1:
        robot.state_machine.payload_size = input;
        robot.state_machine.write_pointer = 0;
        robot.state_machine.state = READING_PAYLOAD; break;
    case READING_PAYLOAD:
        robot.state_machine.buffer[robot.state_machine.write_pointer++] = input;
        robot.state_machine.state = (robot.state_machine.write_pointer == robot.state_machine.payload_size) ? CHECK_CRC : READING_PAYLOAD; break;
    case CHECK_CRC:
        robot.state_machine.buffer[robot.state_machine.write_pointer++] = input;
        if (kobuki_check_crc()){
            robot.state_machine.read_pointer = 0;
#ifdef KOBUKI_DEBUG
            printf("Payload received: ");
            for (size_t i = 0; i <= robot.state_machine.payload_size; i++){ printf("0x%02X ", robot.state_machine.buffer[i]);}
            printf("\n");
#endif
            kobuki_process_payload();
        } else{
            kobuki_crc_error_hook();
        }
        robot.state_machine.state = NO_SYNC; break;
    default:
        break;
    }
}

// **********************************
// *         KOBUKI COMMANDS        *
// **********************************

// Not implemented: https://kobuki.readthedocs.io/en/devel/protocol.html
// Sound Sequence
// Request extra
// GPIO
// Set controller gain
// Get controller gain

void kobuki_set_speed_command(float translation, float rotation){
    pthread_mutex_lock(&robot_lock);
    robot.base_control_command[0] = KOBUKI_BASE_CONTROL_COMMAND;
    robot.base_control_command[1] = KOBUKI_BASE_CONTROL_COMMAND_LEN;

    int16_t * speed = (int16_t *) &robot.base_control_command[2];
    int16_t * radius = (int16_t *) &robot.base_control_command[4];
    
    // kobuki_safety_contrains(&translation, &rotation);
    
    // convert to mm;
    translation *= 1000;
    float b2 = KOBUKI_WHEELBASE * 500;

    if (fabs(rotation) < 1e-3 ) {
        //Pure translation
        *speed = (int16_t) translation;
        *radius = 0;
    } else if (fabs(translation) < 1){
        //Pure rotation
        *radius = 1;
        *speed =  (int16_t) (rotation * b2);
    } else {
        //Translation and rotation
        float r = translation/rotation;
        *radius = (int16_t) r;
        if (r > 1) {
            *speed = (int16_t) (translation * (r + b2)/ r);
        } else if (r < -1) {
            *speed = (int16_t) (translation * (r - b2)/ r);
        }
    }

    #if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
        *speed = *((int16_t *)  kobuki_swap_endianness(speed, 16)));
        *radius = *((int16_t *) kobuki_swap_endianness(radius, 16)));
    #endif
    pthread_mutex_unlock(&robot_lock);

#ifdef KOBUKI_DEBUG
    printf("Setting base control to : translation = %0.3f, rotation = %0.3f\n Hex: ",translation, rotation);
    for (size_t i = 0; i < sizeof(robot.base_control_command); i++){ printf("0x%02X ", robot.base_control_command[i]);}
    printf("\n");
#endif
}

void kobuki_set_sound_command(float frequency, uint8_t duration_ms){
    pthread_mutex_lock(&robot_lock);
    robot.sound_command[0] = KOBUKI_SOUND_COMMAND;
    robot.sound_command[1] = KOBUKI_SOUND_COMMAND_LEN;

    uint16_t * f = (uint16_t *) &robot.sound_command[2];
    *f = (uint16_t) (1.0f / frequency * 0.00000275f);
    robot.sound_command[4] = duration_ms;

    #if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
        *f = *((uint16_t *)  kobuki_swap_endianness(f, 16)));
    #endif
    pthread_mutex_unlock(&robot_lock);

#ifdef KOBUKI_DEBUG
    printf("Setting sound command to : freq = %f, duration = %u ms\n Hex: ",frequency, duration_ms);
    for (size_t i = 0; i < sizeof(robot.sound_command); i++){ printf("0x%02X ", robot.sound_command[i]);}
    printf("\n");
#endif
}


// **********************************
// *       KOBUKI CALLBACKS         *
// **********************************

void kobuki_set_subpayload_callback(kobuki_subpayload_callback_t cb){
    robot.subpayload_callback = cb;
}

void kobuki_set_emergency_callback(kobuki_subpayload_callback_t cb){
    robot.emergency_callback = cb;
}

// **********************************
// *          KOBUKI LOOP           *
// **********************************

#define UART_INPUT_BUFFER_SIZE 200
uint8_t uart_input_buffer[UART_INPUT_BUFFER_SIZE];

void kobuki_loop(){
    kobuki_set_speed_command(0.0, 0.0);
    memset(robot.sound_command, 0, sizeof(robot.sound_command));
    robot.robot_status.odometry.init = false;
    while (1)
    {   
        pthread_mutex_lock(&robot_lock);
        kobuki_write_bytes(robot.base_control_command, sizeof(robot.base_control_command));

        if (robot.sound_command[0] != 0){
            kobuki_write_bytes(robot.sound_command, sizeof(robot.sound_command));
            memset(robot.sound_command, 0, sizeof(robot.sound_command));
        }
        pthread_mutex_unlock(&robot_lock);

        size_t length = kobuki_read_bytes(uart_input_buffer, UART_INPUT_BUFFER_SIZE);
        for (size_t i = 0; i < length; i++){
            kobuki_state_machine(uart_input_buffer[i]);
        }

        usleep(KOBUKI_RATE_MS*1000);
    }
}
