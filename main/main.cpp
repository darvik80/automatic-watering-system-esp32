#include <stdio.h>
#include <hal/gpio_types.h>
#include <driver/gpio.h>
#include <driver/adc.h>
#include <driver/i2c.h>
#include <esp_lcd_types.h>
#include <esp_lcd_panel_io.h>
#include <memory>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"
#include "HD44780.h"
//#include "ssd1306.h"

void i2c_master_init(void) {
    i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = CONFIG_SDA_GPIO,
            .scl_io_num = CONFIG_SCL_GPIO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master = {
                    .clk_speed = 100000,
            }
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));

    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
}

void lcd_send_cmd(char cmd) {
    esp_err_t err;
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data_t[0] = data_u | 0x0C;  //en=1, rs=0
    data_t[1] = data_u | 0x08;  //en=0, rs=0
    data_t[2] = data_l | 0x0C;  //en=1, rs=0
    data_t[3] = data_l | 0x08;  //en=0, rs=0

    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_NUM_0, 0x20, data_t, 4, 1000));
}

void lcd_send_data(char data) {
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    data_t[0] = data_u | 0x0D;  //en=1, rs=0
    data_t[1] = data_u | 0x09;  //en=0, rs=0
    data_t[2] = data_l | 0x0D;  //en=1, rs=0
    data_t[3] = data_l | 0x09;  //en=0, rs=0
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_NUM_0, 0x20, data_t, 4, 1000));
}


void lcd_init() {
    // 4 bit initialisation
    usleep(50000);  // wait for >40ms
    lcd_send_cmd(0x30);
    usleep(4500);  // wait for >4.1ms
    lcd_send_cmd(0x30);
    usleep(200);  // wait for >100us
    lcd_send_cmd(0x30);
    usleep(200);
    lcd_send_cmd(0x20);  // 4bit mode
    usleep(200);

    // dislay initialisation
    lcd_send_cmd(0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
    usleep(1000);
    lcd_send_cmd(0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
    usleep(1000);
    lcd_send_cmd(0x01);  // clear display
    usleep(1000);
    usleep(1000);
    lcd_send_cmd(0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
    usleep(1000);
    lcd_send_cmd(0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
    usleep(2000);
}

void lcd_put_cur(uint8_t col, uint8_t row) {
    switch (row) {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd(col);
}

void lcd_send_string(const char *str) {
    while (*str) lcd_send_data(*str++);
}

void lcd_clear(void) {
    lcd_send_cmd(0x01);
    usleep(5000);
}

static int do_i2cdetect() {
    i2c_master_init();

    uint8_t address;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            fflush(stdout);
            address = i + j;
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 0x01);
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 50 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
            if (ret == ESP_OK) {
                printf("%02x ", address);
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("UU ");
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }

    i2c_driver_delete(I2C_NUM_0);
    return 0;
}

extern "C" {
void app_main(void);
}

void app_main(void) {
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf(
            "This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : ""
    );

    printf("silicon revision %d, ", chip_info.revision);

    printf(
            "%uMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external"
    );

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    do_i2cdetect();

//    SSD1306_t dev;
//    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
//    ssd1306_init(&dev, 128, 64);
//    ssd1306_clear_screen(&dev, false);
//    ssd1306_contrast(&dev, 0xff);
//    ssd1306_display_text(&dev, 0, "Hello\ntest", 10, false);
//    ssd1306_display_text(&dev, 1, "test", 4, false);
//    ssd1306_display_text(&dev, 2, "demo", 4, false);


    i2c_master_init();
    lcd_init();
    lcd_put_cur(0, 0);
    lcd_send_string("Hello");
    lcd_put_cur(0, 1);
    lcd_send_string("World!");
    sleep(5);
    lcd_clear();
    lcd_put_cur(0, 0);
    lcd_send_string("Auto waterling");
    lcd_put_cur(1, 1);
    lcd_send_string("is ready...");

    gpio_set_direction(GPIO_NUM_10, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_10, 0);
//    gpio_set_level(GPIO_NUM_10, 1);
//    printf("High\n");
//    vTaskDelay(5000 / portTICK_PERIOD_MS);
//    gpio_set_level(GPIO_NUM_10, 0);
//    printf("Low\n");

    adc1_channel_t channel{ADC1_CHANNEL_1};
    adc1_config_channel_atten(channel, ADC_ATTEN_DB_11);


    int lastVal = 0;
    for (;;) {
        vTaskDelay(200 / portTICK_PERIOD_MS);
        int val = adc1_get_raw(channel);
        if (val != lastVal) {
            lastVal = val;
            printf("data: %d\n", lastVal);
        }
    }

}
