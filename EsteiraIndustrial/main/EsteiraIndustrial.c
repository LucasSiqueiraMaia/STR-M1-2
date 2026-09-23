#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "driver/touch_sens.h"

#define TAG "ESTEIRA"

#define TP_LOAD  6
#define TP_OBJ   7
#define TP_HMI   8
#define TP_ESTOP 9

#define TOUCH_THRESH_LOAD  400
#define TOUCH_THRESH_OBJ   400
#define TOUCH_THRESH_HMI   400
#define TOUCH_THRESH_ESTOP 400

#define ENC_T_MS 5
#define STK      3072

#define SCHED_POLICY_DM      0
#define SCHED_POLICY_CUSTOM  1
#define SCHED_POLICY_RM      2

#define SCHED_POLICY SCHED_POLICY_CUSTOM

#if SCHED_POLICY == SCHED_POLICY_DM
    #define PRIO_ESTOP 5
    #define PRIO_ENC   5
    #define PRIO_CTRL  3
    #define PRIO_SORT  3
#elif SCHED_POLICY == SCHED_POLICY_RM
    #define PRIO_ENC   5
    #define PRIO_CTRL  4
    #define PRIO_SORT  3
    #define PRIO_ESTOP 2
#else
    #define PRIO_ESTOP 5
    #define PRIO_ENC   4
    #define PRIO_CTRL  3
    #define PRIO_SORT  2
#endif

#define DEADLINE_ENC_US    5000
#define DEADLINE_CTRL_US  10000
#define DEADLINE_SORT_US  10000
#define DEADLINE_ESTOP_US  5000
#define DEADLINE_HMI_US    5000

static TaskHandle_t hENC = NULL, hCTRL = NULL, hSORT = NULL, hSAFE = NULL;

typedef struct {
    int64_t t_evt_us;
} sort_evt_t;

static QueueHandle_t qSort = NULL;
static SemaphoreHandle_t semEStop = NULL;
static SemaphoreHandle_t semHMI   = NULL;

static touch_sensor_handle_t  sens_handle = NULL;
static touch_channel_handle_t chan_load   = NULL;
static touch_channel_handle_t chan_obj    = NULL;
static touch_channel_handle_t chan_hmi    = NULL;
static touch_channel_handle_t chan_estop  = NULL;

static volatile int64_t g_enc_evt_us   = 0;
static volatile int64_t g_hmi_evt_us   = 0;
static volatile int64_t g_estop_evt_us = 0;
static volatile bool    g_load_spike_pending = false;

typedef struct {
    float rpm;
    float pos_mm;
    float set_rpm;
} belt_state_t;

static belt_state_t g_belt = { .rpm = 0.f, .pos_mm = 0.f, .set_rpm = 120.0f };

typedef struct {
    const char *name;
    int32_t  deadline_us;
    uint32_t count;
    uint32_t misses;
} task_metrics_t;

static task_metrics_t g_m_enc   = { "ENC_SENSE", DEADLINE_ENC_US,   0, 0 };
static task_metrics_t g_m_ctrl  = { "SPD_CTRL",  DEADLINE_CTRL_US,  0, 0 };
static task_metrics_t g_m_sort  = { "SORT_ACT",  DEADLINE_SORT_US,  0, 0 };
static task_metrics_t g_m_estop = { "SAFETY",    DEADLINE_ESTOP_US, 0, 0 };
static task_metrics_t g_m_hmi   = { "HMI_SOFT",  DEADLINE_HMI_US,   0, 0 };

static void metrics_log(task_metrics_t *m, int64_t evt_us, int64_t start_us, int64_t end_us)
{
    int64_t latency_us = start_us - evt_us;
    int64_t exec_us    = end_us - start_us;
    int64_t total_us   = end_us - evt_us;
    bool miss = total_us > m->deadline_us;

    m->count++;
    if (miss) m->misses++;

    ESP_LOGI(TAG,
        "%-10s evt=%" PRId64 "us start=%" PRId64 "us end=%" PRId64 "us lat=%" PRId64 "us exec=%" PRId64 "us total=%" PRId64 "us D=%" PRId32 "us %s misses=%" PRIu32 "/%" PRIu32,
        m->name, evt_us, start_us, end_us, latency_us, exec_us, total_us,
        m->deadline_us, miss ? "MISS" : "OK", m->misses, m->count);
}

static inline void cpu_tight_loop_us(uint32_t us)
{
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us) {
        __asm__ __volatile__("nop");
    }
}

static void task_enc_sense(void *arg)
{
    TickType_t next = xTaskGetTickCount();
    const TickType_t T = pdMS_TO_TICKS(ENC_T_MS);

    for (;;) {
        int64_t evt_us   = (int64_t)next * portTICK_PERIOD_MS * 1000;
        int64_t start_us = esp_timer_get_time();

        float err = g_belt.set_rpm - g_belt.rpm;
        g_belt.rpm += 0.05f * err;
        g_belt.pos_mm += (g_belt.rpm / 60.0f) * (ENC_T_MS / 1000.0f) * 100.0f;

        cpu_tight_loop_us(700);

        if (g_load_spike_pending) {
            g_load_spike_pending = false;
            cpu_tight_loop_us(2000);
        }

        g_enc_evt_us = evt_us;
        if (hCTRL) xTaskNotifyGive(hCTRL);

        int64_t end_us = esp_timer_get_time();
        metrics_log(&g_m_enc, evt_us, start_us, end_us);

        vTaskDelayUntil(&next, T);
    }
}

static void task_spd_ctrl(void *arg)
{
    float kp = 0.4f, ki = 0.1f, integ = 0.f;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t evt_us   = g_enc_evt_us;
        int64_t start_us = esp_timer_get_time();

        float err = g_belt.set_rpm - g_belt.rpm;
        integ += err * (ENC_T_MS / 1000.0f);
        float u = kp * err + ki * integ;
        g_belt.set_rpm += 0.1f * u;

        cpu_tight_loop_us(1200);

        int64_t end_us = esp_timer_get_time();
        metrics_log(&g_m_ctrl, evt_us, start_us, end_us);

        if (xSemaphoreTake(semHMI, 0) == pdTRUE) {
            int64_t hmi_evt_us   = g_hmi_evt_us;
            int64_t hmi_start_us = esp_timer_get_time();

            ESP_LOGI(TAG, "HMI rpm=%.1f set=%.1f pos=%.1fmm", g_belt.rpm, g_belt.set_rpm, g_belt.pos_mm);
            cpu_tight_loop_us(400);

            int64_t hmi_end_us = esp_timer_get_time();
            metrics_log(&g_m_hmi, hmi_evt_us, hmi_start_us, hmi_end_us);
        }
    }
}

static void task_sort_act(void *arg)
{
    sort_evt_t ev;
    for (;;) {
        if (xQueueReceive(qSort, &ev, portMAX_DELAY) == pdTRUE) {
            int64_t start_us = esp_timer_get_time();

            cpu_tight_loop_us(800);

            int64_t end_us = esp_timer_get_time();
            metrics_log(&g_m_sort, ev.t_evt_us, start_us, end_us);
        }
    }
}

static void task_safety(void *arg)
{
    for (;;) {
        if (xSemaphoreTake(semEStop, portMAX_DELAY) == pdTRUE) {
            int64_t evt_us   = g_estop_evt_us;
            int64_t start_us = esp_timer_get_time();

            g_belt.set_rpm = 0.f;
            g_belt.rpm     = 0.f;
            cpu_tight_loop_us(900);

            int64_t end_us = esp_timer_get_time();
            metrics_log(&g_m_estop, evt_us, start_us, end_us);
        }
    }
}

static bool IRAM_ATTR touch_on_hw_active_cb(touch_sensor_handle_t sens, const touch_hw_active_event_data_t *event, void *user_ctx)
{
    uint32_t mask = event->active_mask;
    int64_t t_evt = esp_timer_get_time();
    BaseType_t hpw = pdFALSE;

    if (mask & (1U << TP_OBJ)) {
        sort_evt_t ev = { .t_evt_us = t_evt };
        xQueueSendFromISR(qSort, &ev, &hpw);
    }
    if (mask & (1U << TP_HMI)) {
        g_hmi_evt_us = t_evt;
        xSemaphoreGiveFromISR(semHMI, &hpw);
    }
    if (mask & (1U << TP_ESTOP)) {
        g_estop_evt_us = t_evt;
        xSemaphoreGiveFromISR(semEStop, &hpw);
    }
    if (mask & (1U << TP_LOAD)) {
        g_load_spike_pending = true;
    }

    return hpw == pdTRUE;
}

static void touch_driver_setup(void)
{
    touch_sensor_sample_config_t sample_cfg[1] = {
        TOUCH_SENSOR_V1_DEFAULT_SAMPLE_CONFIG(2.0f, TOUCH_VOLT_LIM_L_0V5, TOUCH_VOLT_LIM_H_2V7)
    };
    touch_sensor_config_t sens_cfg = TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(1, sample_cfg);
    ESP_ERROR_CHECK(touch_sensor_new_controller(&sens_cfg, &sens_handle));

    touch_channel_config_t chan_cfg_load = {
        .abs_active_thresh = { TOUCH_THRESH_LOAD },
        .charge_speed      = TOUCH_CHARGE_SPEED_7,
        .init_charge_volt  = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
        .group             = TOUCH_CHAN_TRIG_GROUP_BOTH,
    };
    touch_channel_config_t chan_cfg_obj = {
        .abs_active_thresh = { TOUCH_THRESH_OBJ },
        .charge_speed      = TOUCH_CHARGE_SPEED_7,
        .init_charge_volt  = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
        .group             = TOUCH_CHAN_TRIG_GROUP_BOTH,
    };
    touch_channel_config_t chan_cfg_hmi = {
        .abs_active_thresh = { TOUCH_THRESH_HMI },
        .charge_speed      = TOUCH_CHARGE_SPEED_7,
        .init_charge_volt  = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
        .group             = TOUCH_CHAN_TRIG_GROUP_BOTH,
    };
    touch_channel_config_t chan_cfg_estop = {
        .abs_active_thresh = { TOUCH_THRESH_ESTOP },
        .charge_speed      = TOUCH_CHARGE_SPEED_7,
        .init_charge_volt  = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
        .group             = TOUCH_CHAN_TRIG_GROUP_BOTH,
    };

    ESP_ERROR_CHECK(touch_sensor_new_channel(sens_handle, TP_LOAD,  &chan_cfg_load,  &chan_load));
    ESP_ERROR_CHECK(touch_sensor_new_channel(sens_handle, TP_OBJ,   &chan_cfg_obj,   &chan_obj));
    ESP_ERROR_CHECK(touch_sensor_new_channel(sens_handle, TP_HMI,   &chan_cfg_hmi,   &chan_hmi));
    ESP_ERROR_CHECK(touch_sensor_new_channel(sens_handle, TP_ESTOP, &chan_cfg_estop, &chan_estop));

    touch_event_callbacks_t callbacks = {
        .on_hw_active = touch_on_hw_active_cb,
    };
    ESP_ERROR_CHECK(touch_sensor_register_callbacks(sens_handle, &callbacks, NULL));

    ESP_ERROR_CHECK(touch_sensor_enable(sens_handle));
    ESP_ERROR_CHECK(touch_sensor_start_continuous_scanning(sens_handle));
}

void app_main(void)
{
    qSort    = xQueueCreate(8, sizeof(sort_evt_t));
    semEStop = xSemaphoreCreateBinary();
    semHMI   = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(task_safety,   "SAFETY_TASK", STK, NULL, PRIO_ESTOP, &hSAFE, 0);
    xTaskCreatePinnedToCore(task_spd_ctrl, "SPD_CTRL",    STK, NULL, PRIO_CTRL,  &hCTRL, 0);
    xTaskCreatePinnedToCore(task_enc_sense,"ENC_SENSE",   STK, NULL, PRIO_ENC,   &hENC,  0);
    xTaskCreatePinnedToCore(task_sort_act, "SORT_ACT",    STK, NULL, PRIO_SORT,  &hSORT, 0);

    touch_driver_setup();
}
