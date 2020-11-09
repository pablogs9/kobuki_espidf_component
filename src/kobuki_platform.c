#include <string.h> 
#include <stddef.h>

#include "kobuki.h"
#include "kobuki_platform.h"

#include <driver/uart.h>

// Portability layer for ESP32
static uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
};

void kobuki_init_serial(){
    ESP_ERROR_CHECK(uart_param_config(KOBUKI_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(KOBUKI_UART_PORT, KOBUKI_UART_TXD, KOBUKI_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(KOBUKI_UART_PORT, KOBUKI_UART_BUFFER_SIZE, 0, 0, NULL, 0));
}

void kobuki_write_bytes(uint8_t * buf, size_t len){
#ifdef KOBUKI_DEBUG
    printf("Writing to Kobuki: ");
    for (size_t i = 0; i < len; i++){ printf("0x%02X ", buf[i]);}
    printf("\n");
#endif
    uart_write_bytes(KOBUKI_UART_PORT, (const char*)buf, len);
}

size_t kobuki_read_bytes(uint8_t * buf, size_t len){
    size_t available_lenght;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(KOBUKI_UART_PORT, (size_t*)&available_lenght));
    available_lenght = (available_lenght > len) ? len : available_lenght;
    return uart_read_bytes(KOBUKI_UART_PORT, buf, available_lenght, pdMS_TO_TICKS(KOBUKI_UART_READ_WAIT_MS));
    
    // uint8_t fake_buff[] = {0xAA, 0x55, 12, 10, 4, 1, 2, 3, 4, 11, 4, 8,8,8,8, 0};
    // memcpy(buf, fake_buff, sizeof(fake_buff));
    // return sizeof(fake_buff);
}