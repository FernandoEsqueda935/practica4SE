
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#define SCL_IO_PIN 22
#define SDA_IO_PIN 21
#define MASTER_FREQUENCY CONFIG_I2C_MASTER_FREQUENCY
#define SLAVE_ADDR 0x27
#define PORT_NUMBER -1
#define LENGTH 48

long signed int DIG_T1 = 27698;
long signed int DIG_T2= -410;
long signed int DIG_T3= 189;

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;


void i2c_master_init() {
    i2c_master_bus_config_t i2c_mst_config_1 = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .scl_io_num = SCL_IO_PIN,
        .sda_io_num = SDA_IO_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config_1, &bus_handle));
    
}

void i2c_slaves_devices_init() {
    i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = SLAVE_ADDR,
    .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
}


void app_main(void)
{
    i2c_master_init();
    i2c_slaves_devices_init();

    ESP_ERROR_CHECK(i2c_master_probe(bus_handle, SLAVE_ADDR, -1));
    /*
    uint8_t buf[20] = {0xF4,0x20};
    uint8_t buffer[3];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buf, 2 , buffer, 3, -1));
    */

    /* Configurar y leer los 3 registros de temperatura*/
    uint8_t data_wr[8] = {0x9F};
    
    uint8_t r_buffer[6] = {0 , 0 , 0, 0 , 0 , 0 };

    data_wr[0]= 0x9F;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, data_wr, 1, r_buffer, 4, -1));
   
    int32_t temperature = 0;
    temperature = r_buffer[0] | (r_buffer[1] << 8) | (r_buffer[2] << 16) | (r_buffer[3] << 24);

    printf("Temperature: %d\n", (int) temperature);

    while (1) {
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
