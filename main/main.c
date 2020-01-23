#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "ldl_radio.h"
#include "ldl_mac.h"
#include "ldl_sm.h"
#include "ldl_system.h"

#define SPI_SCLK        GPIO_NUM_18
#define SPI_MISO        GPIO_NUM_19
#define SPI_MOSI        GPIO_NUM_23
#define RFM_NSS         GPIO_NUM_5
#define RFM_RESET       GPIO_NUM_17
#define RFM_DIO0        GPIO_NUM_4
#define RFM_DIO1        GPIO_NUM_16

/* ticks per second (micros()) */
#define TPS 1000000UL

const char *TAG = "main";

static spi_device_handle_t spi_handle;

QueueHandle_t dio_wake_queue;

const uint8_t app_key_ptr[16];
const uint8_t nwk_key_ptr[16];// = { ... };

const uint8_t app_eui[8];// = { ... };
const uint8_t dev_eui[8];// = { ... };

struct ldl_sm sm;
struct ldl_radio radio;
struct ldl_mac mac;

uint32_t LDL_System_ticks(void *app)
{
    return (uint32_t) esp_timer_get_time();
}

uint32_t LDL_System_tps(void)
{
    return TPS;
}

uint32_t LDL_System_eps(void)
{
    return 5000;
}

void LDL_Chip_select(void *self, bool state)
{
    gpio_set_level(RFM_NSS, !state);
}

void LDL_Chip_reset(void *self, bool state)
{
    gpio_set_direction(RFM_RESET, state ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT);
}

void LDL_Chip_write(void *self, uint8_t data)
{
    /* @formatter:off */
    spi_transaction_t t = {
          .length = 8,
          .tx_buffer = &data,
          .rxlength = 8,
          .rx_buffer = &data
    };
    /* @formatter:on */
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &t));
}

uint8_t LDL_Chip_read(void *self)
{
    uint8_t data = 0;
    /* @formatter:off */
    spi_transaction_t t = {
            .length = 8,
            .tx_buffer = &data,
            .rxlength = 8,
            .rx_buffer = &data
    };
    /* @formatter:on */
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &t));
    return data;
}

void app_handler(void *app, enum ldl_mac_response_type type, const union ldl_mac_response_arg *arg)
{
    switch (type) {
        case LDL_MAC_CHIP_ERROR:
            ESP_LOGI(TAG, "LDL_MAC_CHIP_ERROR");
            break;
        case LDL_MAC_RESET:
            ESP_LOGI(TAG, "LDL_MAC_RESET");
            break;
        case LDL_MAC_STARTUP:
            ESP_LOGI(TAG, "LDL_MAC_STARTUP");
            srand(arg->startup.entropy);
            break;
        case LDL_MAC_JOIN_COMPLETE:
            ESP_LOGI(TAG, "LDL_MAC_JOIN_COMPLETE");
            break;
        case LDL_MAC_JOIN_TIMEOUT:
            ESP_LOGI(TAG, "LDL_MAC_JOIN_TIMEOUT");
            break;
        case LDL_MAC_DATA_COMPLETE:
            ESP_LOGI(TAG, "LDL_MAC_DATA_COMPLETE");
            break;
        case LDL_MAC_DATA_TIMEOUT:
            ESP_LOGI(TAG, "LDL_MAC_DATA_TIMEOUT");
            break;
        case LDL_MAC_DATA_NAK:
            ESP_LOGI(TAG, "LDL_MAC_DATA_NAK");
            break;
        case LDL_MAC_RX:
            ESP_LOGI(TAG, "LDL_MAC_RX");
            break;
        case LDL_MAC_LINK_STATUS:
            ESP_LOGI(TAG, "LDL_MAC_LINK_STATUS");
            break;
        case LDL_MAC_RX1_SLOT:
            ESP_LOGI(TAG, "LDL_MAC_RX1_SLOT");
            break;
        case LDL_MAC_RX2_SLOT:
            ESP_LOGI(TAG, "LDL_MAC_RX2_SLOT");
            break;
        case LDL_MAC_DOWNSTREAM:
            ESP_LOGI(TAG, "LDL_MAC_DOWNSTREAM");
            break;
        case LDL_MAC_TX_COMPLETE:
            ESP_LOGI(TAG, "LDL_MAC_TX_COMPLETE");
            break;
        case LDL_MAC_TX_BEGIN:
            ESP_LOGI(TAG, "LDL_MAC_TX_BEGIN");
            break;
        case LDL_MAC_SESSION_UPDATED:
            ESP_LOGI(TAG, "LDL_MAC_SESSION_UPDATED");
            break;
        case LDL_MAC_DEVICE_TIME:
            ESP_LOGI(TAG, "LDL_MAC_DEVICE_TIME");
            break;
        default:
            break;
    }
}

static void IRAM_ATTR dio0_isr_handler(void *arg)
{
    uint8_t dio = 0;
    BaseType_t higherPrioTaskWoken = pdFALSE;
    xQueueSendFromISR(dio_wake_queue, &dio, &higherPrioTaskWoken);
//    if (higherPrioTaskWoken) portYIELD_FROM_ISR();
//    LDL_Radio_interrupt(&radio, 0);
}

static void IRAM_ATTR dio1_isr_handler(void *arg)
{
    uint8_t dio = 1;
    BaseType_t higherPrioTaskWoken = pdFALSE;
    xQueueSendFromISR(dio_wake_queue, &dio, &higherPrioTaskWoken);
//    if (higherPrioTaskWoken) portYIELD_FROM_ISR();
//    LDL_Radio_interrupt(&radio, 1);
}

static void init_spi()
{
    spi_bus_config_t spi_cfg = { 0 };
    spi_cfg.miso_io_num = SPI_MISO;
    spi_cfg.mosi_io_num = SPI_MOSI;
    spi_cfg.sclk_io_num = SPI_SCLK;
    spi_cfg.quadwp_io_num = -1;
    spi_cfg.quadhd_io_num = -1;
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &spi_cfg, 1));

    /* @formatter:off */
    spi_device_interface_config_t spi_dev_cfg = {
        .mode = 0,
        .clock_speed_hz = 10000000,
        .command_bits = 0,
        .address_bits = 0,
        .spics_io_num = -1,
        .queue_size = 1
    };
    /* @formatter:on */
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &spi_dev_cfg, &spi_handle));
}

static void init_gpio()
{
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));

    gpio_pad_select_gpio(RFM_NSS);
    gpio_set_direction(RFM_NSS, GPIO_MODE_OUTPUT);
    gpio_set_level(RFM_NSS, 1);

    gpio_pad_select_gpio(RFM_RESET);
    gpio_set_direction(RFM_RESET, GPIO_MODE_INPUT);
    gpio_set_level(RFM_RESET, 0);

    gpio_pad_select_gpio(RFM_DIO0);
    gpio_set_direction(RFM_DIO0, GPIO_MODE_INPUT);

    gpio_pad_select_gpio(RFM_DIO1);
    gpio_set_direction(RFM_DIO1, GPIO_MODE_INPUT);
}

static void enable_interrupts()
{
    gpio_set_intr_type(RFM_DIO0, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(RFM_DIO0, dio0_isr_handler, NULL);

    gpio_set_intr_type(RFM_DIO1, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(RFM_DIO1, dio1_isr_handler, NULL);
}

void app_main()
{
    ESP_LOGI(TAG, "Starting...");

    init_spi();
    init_gpio();

    dio_wake_queue = xQueueCreate(10, sizeof(uint8_t));

    LDL_SM_init(&sm, app_key_ptr, nwk_key_ptr);

    LDL_Radio_init(&radio, LDL_RADIO_SX1276, NULL);
    LDL_Radio_setPA(&radio, LDL_RADIO_PA_BOOST);

    struct ldl_mac_init_arg arg = { 0 };

    arg.radio = &radio;
    arg.handler = app_handler;
    arg.sm = &sm;
    arg.session = NULL; /* restore cached session state (or not, in this case) */
    arg.devNonce = 0U; /* restore devNonce */
    arg.joinNonce = 0U; /* restore joinNonce */
    arg.gain = 0; /* +/- dBm gain correction  */
    arg.joinEUI = app_eui;
    arg.devEUI = dev_eui;

    LDL_MAC_init(&mac, LDL_EU_863_870, &arg);

    LDL_MAC_setMaxDCycle(&mac, 12);

    enable_interrupts();

    for (;;) {
        if (LDL_MAC_ready(&mac)) {
            if (LDL_MAC_joined(&mac)) {
                const char *data = "Cheeki breeki!";
                LDL_MAC_unconfirmedData(&mac, 1U, data, strlen(data) - 1, NULL);
            } else {
                LDL_MAC_otaa(&mac);
            }
        }

        LDL_MAC_process(&mac);

        uint32_t ticks_until_next_event = LDL_MAC_ticksUntilNextEvent(&mac);
        uint8_t dio;
        if (xQueueReceive(dio_wake_queue, &dio, (ticks_until_next_event / (TPS / 1000)) / portTICK_PERIOD_MS)) {
            LDL_Radio_interrupt(&radio, dio);
        }
    }
}