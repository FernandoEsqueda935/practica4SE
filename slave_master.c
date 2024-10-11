
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "driver/i2c_types.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#define SCL_IO_PIN 22
#define SDA_IO_PIN 21

#define SDA_IO_PIN_SLAVE 4
#define SCL_IO_PIN_SLAVE 5

#define MASTER_FREQUENCY CONFIG_I2C_MASTER_FREQUENCY
#define SLAVE_ADDR 0x76
#define PORT_NUMBER -1
#define LENGTH 48

#define I2C_GET_TEMPERATURE BIT0
#define I2C_TEMPERATURE_READY BIT1

long signed int DIG_T1 = 27698;
long signed int DIG_T2= -410;
long signed int DIG_T3= 189;

int32_t temperature;


i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;
i2c_slave_dev_handle_t slave_dev_handle;

QueueHandle_t s_receive_queue;

EventGroupHandle_t i2c_event_group;



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
    i2c_slave_config_t dev_cfg = {
        .i2c_port = -1,
        .sda_io_num = SDA_IO_PIN,
        .scl_io_num = SCL_IO_PIN, 
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
}

void i2c_slave_init() {
    i2c_slave_config_t i2c_slv_config = {
        .addr_bit_len = I2C_ADDR_BIT_LEN_7,   // 7-bit address
        .clk_source = I2C_CLK_SRC_DEFAULT,    // set the clock source
        .i2c_port = 0,                        // set I2C port number
        .send_buf_depth = 256,                // set tx buffer length
        .scl_io_num = 2,                      // SCL gpio number
        .sda_io_num = 1,                      // SDA gpio number
        .slave_addr = 0x27,                   // slave address
    };
    ESP_ERROR_CHECK(i2c_new_slave_device(&i2c_slv_config, &slave_dev_handle));

}

long signed int BME280_compensate_T_double(long signed int adc_T) {
    long signed int var1, var2;
    var1 = ((((adc_T>>3) - ((long signed int)DIG_T1<<1))) * ((long signed int)DIG_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((long signed int)DIG_T1)) * ((adc_T>>4) - ((long signed int)DIG_T1))) >> 12) * ((long signed int)DIG_T3)) >> 14;
    return var1 + var2;
}

void get_compensation_values() {
    uint8_t r_buffer[6] = {0 , 0 , 0, 0 , 0 , 0 };
    uint8_t data= 0x88;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &data, 1, r_buffer, 6, -1));
    DIG_T1 = r_buffer[0] | r_buffer[1] << 8;
    DIG_T2 = r_buffer[2] | r_buffer[3] << 8;
    DIG_T3 = r_buffer[4] | r_buffer[5] << 8;
}

long signed int get_temperature() {
    //instrucciones necesarias para solo obtener la medicion de temperatura, direccion/valor 
    uint8_t data_wr[8] = {0xF5, 0x00, 0xF2, 0x00, 0xF4, 0xA1};
    uint8_t r_buffer[6] = {0 , 0 , 0, 0 , 0 , 0 };
    
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data_wr, 6, -1));

    data_wr[0]= 0xF3;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, data_wr, 1, r_buffer, 1, -1));

    while (r_buffer[0] & (1<<0|1<<3)) {
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, data_wr, 1, r_buffer, 1, -1));
    }

    data_wr[0]= 0xFA;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, data_wr, 1, r_buffer, 3, -1));

    long signed int adc_T = 0;
    adc_T = r_buffer[0] << 12 | r_buffer[1] << 4 | r_buffer[2] >> 4;

    long signed int t_fine = BME280_compensate_T_double(adc_T);

    long signed int T = (t_fine * 5 + 128) >> 8;

    return T;
}

static IRAM_ATTR bool i2c_slave_rx_done_callback(i2c_slave_dev_handle_t channel, const i2c_slave_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}


void get_temperature_task(void *args) {
    while (1) {
        xEventGroupWaitBits(i2c_event_group, I2C_GET_TEMPERATURE, pdTRUE, pdTRUE, portMAX_DELAY);
        temperature = get_temperature();
        xEventGroupSetBits(i2c_event_group, I2C_TEMPERATURE_READY);
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

void i2c_master_task(void *args) {
    
    uint8_t data_rd = 0;
    i2c_slave_rx_done_event_data_t rx_data;
    
    s_receive_queue = xQueueCreate(1, sizeof(i2c_slave_rx_done_event_data_t));
    i2c_slave_event_callbacks_t cbs = {
        .on_recv_done = i2c_slave_rx_done_callback,
    };
    ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(slave_dev_handle, &cbs, s_receive_queue));

    while (1) {

        
        if (xQueueReceive(s_receive_queue, &rx_data, portMAX_DELAY)) {

            ESP_ERROR_CHECK(i2c_slave_receive(slave_dev_handle, &data_rd, 1));

            if (data_rd == 0x9F) {
                xEventGroupSetBits(i2c_event_group, I2C_GET_TEMPERATURE);
                xEventGroupWaitBits(i2c_event_group, I2C_TEMPERATURE_READY, pdTRUE, pdTRUE, portMAX_DELAY);
                ESP_ERROR_CHECK(i2c_slave_transmit(slave_dev_handle, (uint8_t *)&temperature, 4, -1));
            }
        }
    vTaskDelay(10/portTICK_PERIOD_MS);
    }
}


void app_main(void)
{
    i2c_master_init();
    i2c_slaves_devices_init();
    get_compensation_values();

    i2c_slave_init();

    xTaskCreate(get_temperature_task, "get_temperature_task", 1024, NULL, 5, NULL);
    xTaskCreate(i2c_master_task, "i2c_master_task", 1024, NULL, 5, NULL);
}
