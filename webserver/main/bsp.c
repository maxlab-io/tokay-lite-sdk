#include "bsp.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/temperature_sensor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "ext_rtc.h"

#define BATT_DETECT_ADC_CHANNEL  ADC_CHANNEL_0 // GPIO1
#define BATT_DETECT_MV_TO_BATT_MV(x) (x * 2)

#define I2C_FREQUENCY 100000

#define I2C_SDA_PIN 13
#define I2C_SCL_PIN 3

#define BUTTON_DEBOUNCE_TIME_MS 50
#define BUTTON_PIN              41LLU

#define PWR_EN_PIN              14

#define TAG "bsp"

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t vbat_adc_cali_handle;
static temperature_sensor_handle_t temp_sensor;

static void (*button_cb)(void);
static TimerHandle_t button_debounce_timer;
static bool button_debounce_timer_started;

static bool init_adc_calibration(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void init_vbat_adc(void);
static void init_button(void);

static void button_isr(void *param);
static void button_debounce_timer_handler(TimerHandle_t timer);

void bsp_init(void (*usr_button_cb)(void))
{
    ESP_ERROR_CHECK(i2c_driver_install(BSP_I2C_BUS_ID, I2C_MODE_MASTER, 0, 0, 0));

    if (NULL != usr_button_cb) {
        button_cb = usr_button_cb;
        init_button();
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = I2C_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_FREQUENCY,
    };

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1LLU << PWR_EN_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ESP_ERROR_CHECK(gpio_set_level(PWR_EN_PIN, 1));
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(i2c_param_config(BSP_I2C_BUS_ID, &conf));
    init_vbat_adc();

    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));

    if (!ext_rtc_init(BSP_I2C_BUS_ID)) {
        ESP_LOGE(TAG, "Failed to initialize RTC");
    }
}

uint32_t bsp_read_vbat(void)
{
    int ret = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BATT_DETECT_ADC_CHANNEL, &ret));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(vbat_adc_cali_handle, ret, &ret));
    return BATT_DETECT_MV_TO_BATT_MV(ret);
}

void bsp_temp_sensor_read(float *p_out)
{
    *p_out = 0;
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, p_out));
    ESP_ERROR_CHECK(temperature_sensor_disable(temp_sensor));
}

bool bsp_deep_sleep(uint32_t sleep_duration_seconds)
{
    if (!ext_rtc_clear_alarm_flag()) {
        ESP_LOGE(TAG, "Failed to clear RTC alarm");
        return false;
    }

    if (sleep_duration_seconds < UINT32_MAX) {
        if (!ext_rtc_set_alarm(sleep_duration_seconds)) {
            ESP_LOGE(TAG, "Failed to set RTC alarm");
            return false;
        }
    }

    ESP_LOGI(TAG, "Shutting down");

    ESP_ERROR_CHECK(gpio_set_level(PWR_EN_PIN, 0));

    vTaskDelay(pdMS_TO_TICKS(100));

    return true;
}

bool bsp_get_timer_alarm(void)
{
    bool active = false;
    const bool success = ext_rtc_alarm_active(&active);
    return success && active;
}

bool bsp_get_usr_button_state(void)
{
    return gpio_get_level(BUTTON_PIN);
}

static bool init_adc_calibration(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void init_vbat_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BATT_DETECT_ADC_CHANNEL, &config));

    init_adc_calibration(ADC_UNIT_1, ADC_ATTEN_DB_11, &vbat_adc_cali_handle);
}

static void IRAM_ATTR button_isr(void *param)
{
    if (!button_debounce_timer_started) {
        BaseType_t wakeup_needed = pdFALSE;
        xTimerStartFromISR(button_debounce_timer, &wakeup_needed);
        button_debounce_timer_started = true;
        portYIELD_FROM_ISR(wakeup_needed);
    }
}

static void button_debounce_timer_handler(TimerHandle_t timer)
{
    button_debounce_timer_started = false;
    if (gpio_get_level(BUTTON_PIN) == 0) {
        button_cb();
    }
}

static void init_button(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1LLU << BUTTON_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    button_debounce_timer = xTimerCreate("DEBOUNCE", pdMS_TO_TICKS(BUTTON_DEBOUNCE_TIME_MS),
                             pdFALSE, NULL, button_debounce_timer_handler);
    if (gpio_get_level(BUTTON_PIN) == 0) {
        xTimerStart(button_debounce_timer, portMAX_DELAY);
        button_debounce_timer_started = true;
    }

    gpio_isr_handler_add(BUTTON_PIN, button_isr, NULL);
}
