#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

static const char *TAG = "BLE_SENSOR";

// TODO: Define your service UUID here
#define IMU_SERVICE_UUID  0x1234
// TODO: Define your 6 characteristic UUIDs here
#define ACCEL_X_UUID  0x2001
#define ACCEL_Y_UUID  0x2002
#define ACCEL_Z_UUID  0x2003
#define GYRO_X_UUID   0x2004
#define GYRO_Y_UUID   0x2005
#define GYRO_Z_UUID   0x2006
// LED pin
#define LED_TRANSMIT_PIN  20

// SPI pin definitions
#define SPI_HOST    SPI2_HOST
#define PIN_NUM_MISO 5
#define PIN_NUM_MOSI 6
#define PIN_NUM_CLK  4
#define PIN_NUM_CS   7


// TODO: Define storage for your 6 sensor values here (int16_t arrays)

// Storage for sensor values
static int16_t accel_x = 0;
static int16_t accel_y = 0;
static int16_t accel_z = 0;
static int16_t gyro_x = 0;
static int16_t gyro_y = 0;
static int16_t gyro_z = 0;

// SPI device handle (global so sensor task can use it)
static spi_device_handle_t spi;
// Storage for characteristic handles
static uint16_t accel_x_handle;
static uint16_t accel_y_handle;
static uint16_t accel_z_handle;
static uint16_t gyro_x_handle;
static uint16_t gyro_y_handle;
static uint16_t gyro_z_handle;
// Connection state
static bool client_connected = false;
// TODO: Write the access callback function (handles reads/writes)
static int sensor_access_callback(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        ESP_LOGI(TAG, "Characteristic read, attr_handle=%d", attr_handle);
        
        // Check which characteristic is being read and return the appropriate value
        if (attr_handle == accel_x_handle) {
            rc = os_mbuf_append(ctxt->om, &accel_x, sizeof(accel_x));
        } else if (attr_handle == accel_y_handle) {
            rc = os_mbuf_append(ctxt->om, &accel_y, sizeof(accel_y));
        } else if (attr_handle == accel_z_handle) {
            rc = os_mbuf_append(ctxt->om, &accel_z, sizeof(accel_z));
        } else if (attr_handle == gyro_x_handle) {
            rc = os_mbuf_append(ctxt->om, &gyro_x, sizeof(gyro_x));
        } else if (attr_handle == gyro_y_handle) {
            rc = os_mbuf_append(ctxt->om, &gyro_y, sizeof(gyro_y));
        } else if (attr_handle == gyro_z_handle) {
            rc = os_mbuf_append(ctxt->om, &gyro_z, sizeof(gyro_z));
        } else {
            return BLE_ATT_ERR_UNLIKELY;
        }
        
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}
// TODO: Write the service definition array
// First, we need to convert our 16-bit UUIDs to the format NimBLE expects
static const ble_uuid16_t imu_service_uuid = BLE_UUID16_INIT(IMU_SERVICE_UUID);
static const ble_uuid16_t accel_x_uuid = BLE_UUID16_INIT(ACCEL_X_UUID);
static const ble_uuid16_t accel_y_uuid = BLE_UUID16_INIT(ACCEL_Y_UUID);
static const ble_uuid16_t accel_z_uuid = BLE_UUID16_INIT(ACCEL_Z_UUID);
static const ble_uuid16_t gyro_x_uuid = BLE_UUID16_INIT(GYRO_X_UUID);
static const ble_uuid16_t gyro_y_uuid = BLE_UUID16_INIT(GYRO_Y_UUID);
static const ble_uuid16_t gyro_z_uuid = BLE_UUID16_INIT(GYRO_Z_UUID);

// Now define the service structure
static const struct ble_gatt_svc_def gatt_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &imu_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &accel_x_uuid.u,
                .access_cb = sensor_access_callback,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &accel_x_handle,
            },            
            {
                .uuid = &accel_y_uuid.u,
                .access_cb = sensor_access_callback,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &accel_y_handle,
            },  
            {
                .uuid = &accel_z_uuid.u,
                .access_cb = sensor_access_callback,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &accel_z_handle,
            },  
            {
                .uuid = &gyro_x_uuid.u,
                .access_cb = sensor_access_callback,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &gyro_x_handle,
            }, 
            {
                .uuid = &gyro_y_uuid.u,
                .access_cb = sensor_access_callback,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &gyro_y_handle,
            }, 
            {
                .uuid = &gyro_z_uuid.u,
                .access_cb = sensor_access_callback,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &gyro_z_handle,
            },

            
            { 0 }  // End of characteristics
        },
    },
    { 0 }  // End of services
};

static int ble_gap_event(struct ble_gap_event *event, void *arg);

static void ble_app_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    
    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)"ESP32_Sensor";
    fields.name_len = strlen("ESP32_Sensor");
    fields.name_is_complete = 1;
    
    ble_gap_adv_set_fields(&fields);
    
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "Connection %s",
                 event->connect.status == 0 ? "established" : "failed");
        if (event->connect.status == 0) {
            client_connected = true;} else{ 
            ble_app_advertise();
        }
        break;
        
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected");
          client_connected = false;  
        ble_app_advertise();
        break;
    }
    return 0;
}

static void ble_app_on_sync(void)
{
    ble_app_advertise();
}

static void host_task(void *param)
{
    nimble_port_run();
}
//initialize spi for sensor
static void spi_init(void)
{
    esp_err_t ret;
    
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8
    };
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
    };
    
    ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    
    ret = spi_bus_add_device(SPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "SPI initialized");
}

//wake BMI
static void bmi160_wake(void)
{
    esp_err_t ret;
    
    // Check device ID
    spi_transaction_t t_id;
    memset(&t_id, 0, sizeof(t_id));
    t_id.length = 16;
    t_id.tx_data[0] = 0x00 | 0x80;
    t_id.tx_data[1] = 0x00;
    t_id.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t_id.rxlength = 16;
    spi_device_transmit(spi, &t_id);
    ESP_LOGI(TAG, "BMI160 Device ID: 0x%02X", t_id.rx_data[1]);
    
    // Wake accelerometer
    spi_transaction_t t_wake_accel;
    memset(&t_wake_accel, 0, sizeof(t_wake_accel));
    t_wake_accel.length = 16;
    t_wake_accel.tx_data[0] = 0x7E;
    t_wake_accel.tx_data[1] = 0x11;
    t_wake_accel.flags = SPI_TRANS_USE_TXDATA;
    spi_device_transmit(spi, &t_wake_accel);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Wake gyroscope
    spi_transaction_t t_wake_gyro;
    memset(&t_wake_gyro, 0, sizeof(t_wake_gyro));
    t_wake_gyro.length = 16;
    t_wake_gyro.tx_data[0] = 0x7E;
    t_wake_gyro.tx_data[1] = 0x15;
    t_wake_gyro.flags = SPI_TRANS_USE_TXDATA;
    spi_device_transmit(spi, &t_wake_gyro);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "BMI160 powered on");
}
//create the task used to read the sensor
static void sensor_task(void *param)
{
    uint8_t tx_gyro_buf[7] = {0x0C | 0x80, 0, 0, 0, 0, 0, 0};
    uint8_t rx_gyro_buf[7] = {0};
    uint8_t tx_accel_buf[7] = {0x12 | 0x80, 0, 0, 0, 0, 0, 0};
    uint8_t rx_accel_buf[7] = {0};
    
    while (1) {
        // Read gyro
        spi_transaction_t t_gyro;
        memset(&t_gyro, 0, sizeof(t_gyro));
        t_gyro.length = 56;
        t_gyro.tx_buffer = tx_gyro_buf;
        t_gyro.rx_buffer = rx_gyro_buf;
        t_gyro.rxlength = 56;
        spi_device_transmit(spi, &t_gyro);
        
        // Read accel
        spi_transaction_t t_accel;
        memset(&t_accel, 0, sizeof(t_accel));
        t_accel.length = 56;
        t_accel.tx_buffer = tx_accel_buf;
        t_accel.rx_buffer = rx_accel_buf;
        t_accel.rxlength = 56;
        spi_device_transmit(spi, &t_accel);
        
        // Update global BLE variables
        gyro_x = (int16_t)((rx_gyro_buf[2] << 8) | rx_gyro_buf[1]);
        gyro_y = (int16_t)((rx_gyro_buf[4] << 8) | rx_gyro_buf[3]);
        gyro_z = (int16_t)((rx_gyro_buf[6] << 8) | rx_gyro_buf[5]);
        
        accel_x = (int16_t)((rx_accel_buf[2] << 8) | rx_accel_buf[1]);
        accel_y = (int16_t)((rx_accel_buf[4] << 8) | rx_accel_buf[3]);
        accel_z = (int16_t)((rx_accel_buf[6] << 8) | rx_accel_buf[5]);
        
   


   if (client_connected) {
    ble_gatts_chr_updated(accel_x_handle);
    ble_gatts_chr_updated(accel_y_handle);
    ble_gatts_chr_updated(accel_z_handle);
    ble_gatts_chr_updated(gyro_x_handle);
    ble_gatts_chr_updated(gyro_y_handle);
    ble_gatts_chr_updated(gyro_z_handle);
  
       gpio_set_level(LED_TRANSMIT_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LED_TRANSMIT_PIN, 0);
}

// Blink LED only if notifications were sent


        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    nvs_flash_init();

    gpio_reset_pin(LED_TRANSMIT_PIN);
    gpio_set_direction(LED_TRANSMIT_PIN,GPIO_MODE_OUTPUT);
    gpio_set_level(LED_TRANSMIT_PIN,0);
    ESP_LOGI(TAG, "LED initialized on GPIO %d", LED_TRANSMIT_PIN);
    
    nimble_port_init();
    
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    
    // TODO: Register your GATT services here
    int rc = ble_gatts_count_cfg(gatt_services);
if (rc != 0) {
    ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
    return;
}

rc = ble_gatts_add_svcs(gatt_services);
if (rc != 0) {
    ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
    return;
}

ESP_LOGI(TAG, "Services registered");
    
    ble_svc_gap_init();
    ble_svc_gatt_init();
    // Initialize SPI and BMI160
spi_init();
bmi160_wake();

// Start sensor reading task
xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    
    nimble_port_freertos_init(host_task);
}