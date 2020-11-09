
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <kobuki.h>

void kobuki_callback(kobuki_subpayload_t * msg){
    switch (msg->type){
        case KOBUKI_BASIC_SENSOR_DATA:
            print_kobuki_basic_sensor_data(&msg->basic_sensor_data);
            break;
        case KOBUKI_DOCKING_IR:
            print_kobuki_docking_ir(&msg->docking_ir);
            break;
        case KOBUKI_INERTIAL_SENSOR_DATA:
            print_kobuki_inertial_sensor_data(&msg->inertial_sensor_data);
            break;
        case KOBUKI_CLIFF_SENSOR_DATA:
            print_kobuki_cliff_sensor_data(&msg->cliff_sensor_data);
            break;
        case KOBUKI_CURRENT_DATA:
            print_kobuki_current_data(&msg->current_data);
            break;
        case KOBUKI_HARDWARE_VERSION:
            print_kobuki_hardware_version(&msg->hardware_version);
            break;
        case KOBUKI_FIRMWARE_VERSION:
            print_kobuki_firmware_version(&msg->firmware_version);
            break;
        case KOBUKI_GPIO_DATA:
            print_kobuki_gpio_data(&msg->gpio_data);
            break;
        case KOBUKI_UDID_DATA:
            print_kobuki_udid_data(&msg->udid_data);
            break;
        case KOBUKI_CONTROLLER_INFO:
            print_kobuki_controller_info(&msg->controller_info);
            break;
        default:
            break;
    }
}

void kobuki_emergency(kobuki_subpayload_t * msg){
    kobuki_set_speed_command(0.0, 0.0);
    printf("EMERGENCY STOP\n");
}

void app_main(void)
{
    kobuki_init_serial();
    kobuki_set_subpayload_callback(kobuki_callback);
    kobuki_set_emergency_callback(kobuki_callback);
    
    TaskHandle_t kobuki_handle = NULL;
    xTaskCreate(kobuki_loop, "kobuki_thread", 2000, NULL,  5, &kobuki_handle); 

    kobuki_set_speed_command(1.0, 0.0);
    while (1){
        printf("Main loop\n");
        sleep(1);
    }
}
