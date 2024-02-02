/* *****************************************************************************
 * File:   drv_dht11.c
 * Author: XX
 *
 * Created on YYYY MM DD
 * 
 * Description: ...
 * 
 **************************************************************************** */

/* *****************************************************************************
 * Header Includes
 **************************************************************************** */
#include "drv_dht11.h"

#include "esp_timer.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* *****************************************************************************
 * Configuration Definitions
 **************************************************************************** */
#define TAG "drv_dht11"

/* *****************************************************************************
 * Constants and Macros Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Enumeration Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Type Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Function-Like Macros
 **************************************************************************** */

/* *****************************************************************************
 * Variables Definitions
 **************************************************************************** */
static gpio_num_t dht_gpio;
static int64_t last_read_time = -2000000;
static struct drv_dht11_reading last_read;

/* *****************************************************************************
 * Prototype of functions definitions
 **************************************************************************** */

/* *****************************************************************************
 * Functions
 **************************************************************************** */

static int _waitOrTimeout(uint16_t microSeconds, int level) {
    int micros_ticks = 0;
    while(gpio_get_level(dht_gpio) == level) { 
        if(micros_ticks++ > microSeconds) 
            return DRV_DHT11_TIMEOUT_ERROR;
        ets_delay_us(1);
    }
    return micros_ticks;
}

static int _checkCRC(uint8_t data[]) {
    if(data[4] == (data[0] + data[1] + data[2] + data[3]))
        return DRV_DHT11_OK;
    else
        return DRV_DHT11_CRC_ERROR;
}

static void _sendStartSignal() {
    gpio_set_direction(dht_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(dht_gpio, 0);
    ets_delay_us(20 * 1000);
    gpio_set_level(dht_gpio, 1);
    ets_delay_us(40);
    gpio_set_direction(dht_gpio, GPIO_MODE_INPUT);
}

static int _checkResponse() {
    /* Wait for next step ~80us*/
    if(_waitOrTimeout(80, 0) == DRV_DHT11_TIMEOUT_ERROR)
        return DRV_DHT11_TIMEOUT_ERROR;

    /* Wait for next step ~80us*/
    if(_waitOrTimeout(80, 1) == DRV_DHT11_TIMEOUT_ERROR) 
        return DRV_DHT11_TIMEOUT_ERROR;

    return DRV_DHT11_OK;
}

static struct drv_dht11_reading _timeoutError() {
    struct drv_dht11_reading timeoutError = {DRV_DHT11_TIMEOUT_ERROR, -1, -1};
    return timeoutError;
}

static struct drv_dht11_reading _crcError() {
    struct drv_dht11_reading crcError = {DRV_DHT11_CRC_ERROR, -1, -1};
    return crcError;
}

void drv_dht11_init(gpio_num_t gpio_num) {
    /* Wait 1 seconds to make the device pass its initial unstable status */
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    dht_gpio = gpio_num;
}

struct drv_dht11_reading drv_dht11_read() {
    /* Tried to sense too soon since last read (dht11 needs ~2 seconds to make a new read) */
    if(esp_timer_get_time() - 2000000 < last_read_time) {
        return last_read;
    }

    last_read_time = esp_timer_get_time();

    uint8_t data[5] = {0,0,0,0,0};

    _sendStartSignal();

    if(_checkResponse() == DRV_DHT11_TIMEOUT_ERROR)
        return last_read = _timeoutError();
    
    /* Read response */
    for(int i = 0; i < 40; i++) {
        /* Initial data */
        if(_waitOrTimeout(50, 0) == DRV_DHT11_TIMEOUT_ERROR)
            return last_read = _timeoutError();
                
        if(_waitOrTimeout(70, 1) > 28) {
            /* Bit received was a 1 */
            data[i/8] |= (1 << (7-(i%8)));
        }
    }

    if(_checkCRC(data) != DRV_DHT11_CRC_ERROR) {
        last_read.status = DRV_DHT11_OK;
        last_read.temperature = data[2];
        last_read.humidity = data[0];
        return last_read;
    } else {
        return last_read = _crcError();
    }
}