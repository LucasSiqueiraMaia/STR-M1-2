/*
 * Esteira Industrial - ESP32 + FreeRTOS (ESP-IDF 5.5)
 * Avaliacao M1 - Sistemas em Tempo Real (UNIVALI)
 *
 * Entradas (touch):
 *   A = T6 (GPIO14) -> pico de carga (nao e gatilho principal)
 *   B = T7 (GPIO27) -> deteccao de objeto  -> SORT_ACT
 *   C = T8 (GPIO33) -> HMI/telemetria      -> tratada dentro de SPD_CTRL (soft)
 *   D = T9 (GPIO32) -> E-STOP              -> SAFETY_TASK
 *
 * Tasks:
 *   ENC_SENSE  (periodica, T=5ms, D=5ms)          hard
 *   SPD_CTRL   (encadeada por ENC_SENSE, D=10ms)  hard
 *   SORT_ACT   (evento touch B, D=10ms)           hard
 *   SAFETY     (evento touch D via ISR, D=5ms)    hard
 *   HMI_SOFT   (evento touch C, D=5ms)            soft
 *   REPORT     (log, prioridade minima)           sem requisito temporal
 *
 * IMPORTANTE (sdkconfig): CONFIG_FREERTOS_HZ=1000 e CONFIG_FREERTOS_UNICORE=y
 * Calibre TOUCH_THRESH_* lendo touch_pad_read() sem e com o dedo no pad.
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "driver/touch_sensor_legacy.h"

#define TAG "ESTEIRA"

#if configTICK_RATE_HZ < 1000
#error "Defina CONFIG_FREERTOS_HZ=1000 (menuconfig -> Component config -> FreeRTOS -> Kernel)."
#endif

#define TP_LOAD   TOUCH_PAD_NUM6   /* A */
#define TP_OBJ    TOUCH_PAD_NUM7   /* B */
#define TP_HMI    TOUCH_PAD_NUM8   /* C */
#define TP_ESTOP  TOUCH_PAD_NUM9   /* D */

#define TOUCH_THRESH_LOAD  400
#define TOUCH_THRESH_OBJ   400
#define TOUCH_THRESH_HMI   400
#define TOUCH_THRESH_ESTOP 400
#define TOUCH_DEBOUNCE_US  500000

#define ENC_T_MS  5
#define STK       4096
#define REPORT_PERIOD_US  1000000

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
#else /* CUSTOM: seguranca acima de tudo */
    #define PRIO_ESTOP 5
    #define PRIO_ENC   4
    #define PRIO_CTRL  3
    #define PRIO_SORT  2
#endif
#define PRIO_REPORT 1

#define DEADLINE_ENC_US    5000
#define DEADLINE_CTRL_US  10000
#define DEADLINE_SORT_US  10000
#define DEADLINE_ESTOP_US  5000
#define DEADLINE_HMI_US    5000

typedef struct {
    const char *name;
    bool     hard;
    int32_t  deadline_us;
    uint32_t count;
    uint32_t misses;
    int64_t  sum_lat, min_lat, max_lat;
    int64_t  max_exec, max_total;
    int64_t  busy_us;
} task_metrics_t;

#define M_INIT(n, h, d) { n, h, d, 0, 0, 0, INT64_MAX, 0, 0, 0, 0 }
static task_metrics_t g_m_enc   = M_INIT("ENC_SENSE", true,  DEADLINE_ENC_US);
static task_metrics_t g_m_ctrl  = M_INIT("SPD_CTRL",  true,  DEADLINE_CTRL_US);
static task_metrics_t g_m_sort  = M_INIT("SORT_ACT",  true,  DEADLINE_SORT_US);
static task_metrics_t g_m_estop = M_INIT("SAFETY",    true,  DEADLINE_ESTOP_US);
static task_metrics_t g_m_hmi   = M_INIT("HMI_SOFT",  false, DEADLINE_HMI_US);

static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    task_metrics_t *m;
    int64_t evt_us, start_us, end_us;
    bool miss;
} log_item_t;

static QueueHandle_t qLog = NULL;
static volatile uint32_t g_log_drops = 0;

static void metrics_record(task_metrics_t *m, int64_t evt_us, int64_t start_us,
                           int64_t end_us, bool always_log)
{
    int64_t lat   = start_us - evt_us;
    if (lat < 0) lat = 0;
    int64_t exec  = end_us - start_us;
    int64_t total = end_us - evt_us;
    bool miss = total > m->deadline_us;

    portENTER_CRITICAL(&g_mux);
    m->count++;
    if (miss) m->misses++;
    m->sum_lat += lat;
    if (lat < m->min_lat) m->min_lat = lat;
    if (lat > m->max_lat) m->max_lat = lat;
    if (exec > m->max_exec) m->max_exec = exec;
    if (total > m->max_total) m->max_total = total;
    m->busy_us += exec;
    portEXIT_CRITICAL(&g_mux);

    if (always_log || miss) {
        log_item_t it = { m, evt_us, start_us, end_us, miss };
        if (xQueueSend(qLog, &it, 0) != pdTRUE) g_log_drops++;
    }
}

static inline void cpu_tight_loop_us(uint32_t us)
{
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us) {
        __asm__ __volatile__("nop");
    }
}

static const float TARGET_RPM = 120.0f;
static volatile float g_rpm = 0.f, g_pos_mm = 0.f, g_pwm = 0.f;
static volatile bool  g_estop = false;
static volatile int64_t g_estop_us = 0;
static volatile bool  g_load_spike_pending = false;

static volatile bool  g_hmi_pending = false;
static volatile float g_hmi_rpm, g_hmi_pwm, g_hmi_pos;

typedef struct { int64_t t_evt_us; } sort_evt_t;

static QueueHandle_t qSort = NULL;
static QueueHandle_t qEnc  = NULL;
static SemaphoreHandle_t semEStop = NULL;
static SemaphoreHandle_t semHMI   = NULL;
static volatile int64_t g_hmi_evt_us = 0, g_estop_evt_us = 0;
static volatile uint32_t g_sort_drops = 0;

static TaskHandle_t hENC = NULL, hCTRL = NULL, hSORT = NULL, hSAFE = NULL, hREP = NULL;

static void task_enc_sense(void *arg)
{
    const TickType_t T = pdMS_TO_TICKS(ENC_T_MS);
    TickType_t last = xTaskGetTickCount();

    vTaskDelayUntil(&last, T);
    int64_t release_us = esp_timer_get_time();

    for (;;) {
        int64_t start_us = esp_timer_get_time();

        float rpm = g_rpm;
        rpm += 0.05f * (g_pwm - rpm);
        g_rpm = rpm;
        g_pos_mm += (rpm / 60.0f) * (ENC_T_MS / 1000.0f) * 100.0f;

        cpu_tight_loop_us(700);

        if (g_load_spike_pending) {
            g_load_spike_pending = false;
            cpu_tight_loop_us(2000);
        }

        xQueueSend(qEnc, &release_us, 0);

        int64_t end_us = esp_timer_get_time();
        metrics_record(&g_m_enc, release_us, start_us, end_us, false);

        release_us += ENC_T_MS * 1000;
        vTaskDelayUntil(&last, T);
    }
}

static void task_spd_ctrl(void *arg)
{
    const float kp = 0.5f, ki = 3.0f, dt = ENC_T_MS / 1000.0f;
    float integ = 0.f;
    int64_t evt_us;

    for (;;) {
        if (xQueueReceive(qEnc, &evt_us, portMAX_DELAY) != pdTRUE) continue;
        int64_t start_us = esp_timer_get_time();

        if (g_estop) {
            integ = 0.f;
            g_pwm = 0.f;
        } else {
            float err = TARGET_RPM - g_rpm;
            integ += err * dt;
            if (integ > 100.f) integ = 100.f;
            if (integ < 0.f)   integ = 0.f;
            float u = kp * err + ki * integ;
            if (u > 250.f) u = 250.f;
            if (u < 0.f)   u = 0.f;
            g_pwm = u;
        }

        cpu_tight_loop_us(1200);

        int64_t end_us = esp_timer_get_time();
        metrics_record(&g_m_ctrl, evt_us, start_us, end_us, false);

        if (xSemaphoreTake(semHMI, 0) == pdTRUE) {
            int64_t h_evt   = g_hmi_evt_us;
            int64_t h_start = esp_timer_get_time();

            g_hmi_rpm = g_rpm; g_hmi_pwm = g_pwm; g_hmi_pos = g_pos_mm;
            g_hmi_pending = true;
            cpu_tight_loop_us(400);

            int64_t h_end = esp_timer_get_time();
            metrics_record(&g_m_hmi, h_evt, h_start, h_end, true);
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
            metrics_record(&g_m_sort, ev.t_evt_us, start_us, end_us, true);
        }
    }
}

static void task_safety(void *arg)
{
    for (;;) {
        if (xSemaphoreTake(semEStop, portMAX_DELAY) == pdTRUE) {
            int64_t evt_us   = g_estop_evt_us;
            int64_t start_us = esp_timer_get_time();

            g_pwm = 0.f;
            g_estop_us = start_us;
            g_estop = true;
            cpu_tight_loop_us(900);

            int64_t end_us = esp_timer_get_time();
            metrics_record(&g_m_estop, evt_us, start_us, end_us, true);
        }
    }
}

static volatile bool    g_pressed[4]  = { false, false, false, false };
static volatile int64_t g_last_evt[4] = { 0, 0, 0, 0 };

static inline int IRAM_ATTR chan_slot(touch_pad_t id)
{
    switch (id) {
        case TP_LOAD:  return 0;
        case TP_OBJ:   return 1;
        case TP_HMI:   return 2;
        case TP_ESTOP: return 3;
        default:       return -1;
    }
}

static void IRAM_ATTR touch_isr_handler(void *arg)
{
    uint32_t status = touch_pad_get_status();
    touch_pad_clear_status();
    int64_t t = esp_timer_get_time();
    BaseType_t hpw = pdFALSE;

    const touch_pad_t chans[4] = { TP_LOAD, TP_OBJ, TP_HMI, TP_ESTOP };
    for (int i = 0; i < 4; i++) {
        touch_pad_t id = chans[i];
        if (!(status & (1U << id))) continue;

        int s = chan_slot(id);
        if (g_pressed[s] && (t - g_last_evt[s]) < TOUCH_DEBOUNCE_US) continue;
        g_pressed[s]  = true;
        g_last_evt[s] = t;

        switch (id) {
        case TP_OBJ: {
            sort_evt_t e = { .t_evt_us = t };
            if (xQueueSendFromISR(qSort, &e, &hpw) != pdTRUE) g_sort_drops++;
            break;
        }
        case TP_HMI:
            g_hmi_evt_us = t;
            xSemaphoreGiveFromISR(semHMI, &hpw);
            break;
        case TP_ESTOP:
            g_estop_evt_us = t;
            xSemaphoreGiveFromISR(semEStop, &hpw);
            break;
        case TP_LOAD:
            g_load_spike_pending = true;
            break;
        default:
            break;
        }
    }

    if (hpw == pdTRUE) portYIELD_FROM_ISR();
}

static void touch_driver_setup(void)
{
    touch_pad_init();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_set_trigger_mode(TOUCH_TRIGGER_BELOW);

    touch_pad_config(TP_LOAD,  TOUCH_THRESH_LOAD);
    touch_pad_config(TP_OBJ,   TOUCH_THRESH_OBJ);
    touch_pad_config(TP_HMI,   TOUCH_THRESH_HMI);
    touch_pad_config(TP_ESTOP, TOUCH_THRESH_ESTOP);

    touch_pad_isr_register(touch_isr_handler, NULL);
    touch_pad_intr_enable();

    ESP_LOGI(TAG, "Touch armado: A=T6(GPIO14) B=T7(GPIO27) C=T8(GPIO33) D=T9(GPIO32)");
}

static void print_summary(int64_t window_us)
{
    static int64_t prev_busy[5];
    task_metrics_t *all[5] = { &g_m_enc, &g_m_ctrl, &g_m_sort, &g_m_estop, &g_m_hmi };
    float total_cpu = 0.f;

    for (int i = 0; i < 5; i++) {
        task_metrics_t s;
        portENTER_CRITICAL(&g_mux);
        s = *all[i];
        portEXIT_CRITICAL(&g_mux);

        float cpu = 100.0f * (float)(s.busy_us - prev_busy[i]) / (float)window_us;
        prev_busy[i] = s.busy_us;
        total_cpu += cpu;

        if (s.count == 0) {
            ESP_LOGI(TAG, "[%-9s %s] sem execucoes  cpu=%.1f%%", s.name, s.hard ? "HARD" : "SOFT", cpu);
            continue;
        }
        ESP_LOGI(TAG,
            "[%-9s %s] n=%" PRIu32 " miss=%" PRIu32 " (%.2f%%) lat avg/max=%" PRId64 "/%" PRId64
            "us jitter=%" PRId64 "us exec max=%" PRId64 "us total max=%" PRId64 "us D=%" PRId32 "us cpu=%.1f%%",
            s.name, s.hard ? "HARD" : "SOFT", s.count, s.misses,
            100.0f * (float)s.misses / (float)s.count,
            s.sum_lat / (int64_t)s.count, s.max_lat, s.max_lat - s.min_lat,
            s.max_exec, s.max_total, s.deadline_us, cpu);
    }
    ESP_LOGI(TAG, "CPU (trabalho das tasks)=%.1f%%  estop=%s  sort_drops=%" PRIu32 " log_drops=%" PRIu32
                  "  rpm=%.1f pwm=%.1f pos=%.1fmm",
             total_cpu, g_estop ? "ATIVO" : "off", g_sort_drops, g_log_drops,
             g_rpm, g_pwm, g_pos_mm);
}

static void task_report(void *arg)
{
    log_item_t it;
    int64_t last_sum = esp_timer_get_time();

    for (;;) {
        if (xQueueReceive(qLog, &it, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_LOGI(TAG,
                "%-9s evt=%" PRId64 " start=%" PRId64 " end=%" PRId64 " | lat=%" PRId64
                "us exec=%" PRId64 "us total=%" PRId64 "us D=%" PRId32 "us %s",
                it.m->name, it.evt_us, it.start_us, it.end_us,
                it.start_us - it.evt_us, it.end_us - it.start_us, it.end_us - it.evt_us,
                it.m->deadline_us, it.miss ? "MISS" : "OK");
            if (it.m == &g_m_estop) {
                ESP_LOGW(TAG, "*** E-STOP: PWM zerado, alarme ativo ***");
            }
        }

        if (g_hmi_pending) {
            g_hmi_pending = false;
            ESP_LOGI(TAG, "HMI rpm=%.1f pwm=%.1f pos=%.1fmm", g_hmi_rpm, g_hmi_pwm, g_hmi_pos);
        }

        int64_t now = esp_timer_get_time();

        if (g_estop && (now - g_estop_us) > 2000000) {
            g_estop = false;
            ESP_LOGW(TAG, "E-STOP rearmado (esteira volta a operar)");
        }

        if ((now - last_sum) >= REPORT_PERIOD_US) {
            print_summary(now - last_sum);
            last_sum = now;
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Politica=%d  CPU=%d MHz  tick=%d Hz  slicing=%d",
             SCHED_POLICY, CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ, (int)configTICK_RATE_HZ,
             (int)configUSE_TIME_SLICING);

    qLog     = xQueueCreate(32, sizeof(log_item_t));
    qSort    = xQueueCreate(8,  sizeof(sort_evt_t));
    qEnc     = xQueueCreate(8,  sizeof(int64_t));
    semEStop = xSemaphoreCreateBinary();
    semHMI   = xSemaphoreCreateBinary();
    configASSERT(qLog && qSort && qEnc && semEStop && semHMI);

    xTaskCreatePinnedToCore(task_safety,    "SAFETY",    STK, NULL, PRIO_ESTOP,  &hSAFE, 0);
    xTaskCreatePinnedToCore(task_spd_ctrl,  "SPD_CTRL",  STK, NULL, PRIO_CTRL,   &hCTRL, 0);
    xTaskCreatePinnedToCore(task_sort_act,  "SORT_ACT",  STK, NULL, PRIO_SORT,   &hSORT, 0);
    xTaskCreatePinnedToCore(task_report,    "REPORT",    STK, NULL, PRIO_REPORT, &hREP,  0);
    xTaskCreatePinnedToCore(task_enc_sense, "ENC_SENSE", STK, NULL, PRIO_ENC,    &hENC,  0);

    touch_driver_setup();
}
