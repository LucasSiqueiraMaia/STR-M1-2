// Esteira Industrial (ESP32 + FreeRTOS) — 4 tasks com touch ISR
// - ENC_SENSE (periódica 5 ms) -> notifica SPD_CTRL
// - SPD_CTRL (hard RT) -> controle PI simulado, trata HMI (soft) se solicitado
// - SORT_ACT (hard RT, evento Touch B) -> aciona "desviador"
// - SAFETY_TASK (hard RT, evento Touch D) -> E-stop
// Compilado com ESP-IDF 5.x

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h"

#define TAG "ESTEIRA"

// ====== Mapeamento dos touch pads ======
#define TP_OBJ   TOUCH_PAD_NUM7   // Touch B -> detecção de objeto (desvio)
#define TP_HMI   TOUCH_PAD_NUM8   // Touch C -> HMI/telemetria
#define TP_ESTOP TOUCH_PAD_NUM9   // Touch D -> E-stop

// ====== Periodicidade, prioridades, stack ======
#define ENC_T_MS        5
#define PRIO_ESTOP      5
#define PRIO_ENC        4
#define PRIO_CTRL       3
#define PRIO_SORT       3
#define STK             3072

// ====== Handles/IPC ======
static TaskHandle_t hENC = NULL, hCTRL = NULL, hSORT = NULL, hSAFE = NULL;

// Notificação ENC->CTRL (direct-to-task notification)
static TaskHandle_t hCtrlNotify = NULL;

// Fila de eventos para o SORT_ACT
typedef struct {
    int64_t t_evt_us;   // timestamp do toque (para logging/latência)
} sort_evt_t;
static QueueHandle_t qSort = NULL;

// Semáforos para E-stop e HMI
static SemaphoreHandle_t semEStop = NULL;
static SemaphoreHandle_t semHMI   = NULL;

// Estado "simulado" da esteira
typedef struct {
    float rpm;
    float pos_mm;
    float set_rpm;
} belt_state_t;

static belt_state_t g_belt = { .rpm = 0.f, .pos_mm = 0.f, .set_rpm = 120.0f };

// ====== Util: busy loop previsível (~WCET) ======
static inline void cpu_tight_loop_us(uint32_t us)
{
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us) {
        __asm__ __volatile__("nop");
    }
}

// ====== ENC_SENSE (periódica 5 ms): estima velocidade/posição ======
static void task_enc_sense(void *arg)
{
    TickType_t next = xTaskGetTickCount();
    const TickType_t T = pdMS_TO_TICKS(ENC_T_MS);

    for (;;) {
        // Dinâmica simulada: aproxima rpm do setpoint, integra posição
        float err = g_belt.set_rpm - g_belt.rpm;
        g_belt.rpm += 0.05f * err;             // aproximação lenta para simular inércia
        g_belt.pos_mm += (g_belt.rpm / 60.0f) * (ENC_T_MS / 1000.0f) * 100.0f; // 100 mm por rev (exemplo)

        // Carga determinística ~0.7 ms
        cpu_tight_loop_us(700);

        // Notifica controle
        if (hCtrlNotify) xTaskNotifyGive(hCtrlNotify);

        vTaskDelayUntil(&next, T);
    }
}

// ====== SPD_CTRL (encadeada): controle PI simulado + HMI (soft) ======
static void task_spd_ctrl(void *arg)
{
    // um controlador PI minimalista e simulado
    float kp = 0.4f, ki = 0.1f, integ = 0.f;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // acorda após ENC_SENSE

        // Controle (hard): alvo é g_belt.set_rpm, atuando em g_belt.rpm (simulado)
        float err = g_belt.set_rpm - g_belt.rpm;
        integ += err * (ENC_T_MS / 1000.0f);
        float u = kp * err + ki * integ;
        // "Aplicar" u: aqui apenas ajustamos o setpoint ligeiramente (mock)
        g_belt.set_rpm += 0.1f * u;

        // Carga determinística ~1.2 ms
        cpu_tight_loop_us(1200);

        // Trecho não-crítico (soft): se HMI solicitada, imprime e retorna rápido
        if (xSemaphoreTake(semHMI, 0) == pdTRUE) {
            printf(TAG, "HMI: rpm=%.1f set=%.1f pos=%.1fmm",g_belt.rpm, g_belt.set_rpm, g_belt.pos_mm);
            cpu_tight_loop_us(400); // ~0.4 ms (soft)
        }
    }
}

// ====== SORT_ACT (evento Touch B): aciona "desviador" no tempo certo ======
static void task_sort_act(void *arg)
{
    sort_evt_t ev;
    for (;;) {
        if (xQueueReceive(qSort, &ev, portMAX_DELAY) == pdTRUE) {
            int64_t t0 = esp_timer_get_time();

            // Preparação de janela crítica (ex.: 0.7 ms)
            cpu_tight_loop_us(700);

            // Aqui você acionaria GPIO/solenóide no "instante"
            // Nesta demo, apenas logamos o tempo gasto:
            int64_t dt = esp_timer_get_time() - t0;
        }
    }
}

// ====== SAFETY_TASK (evento Touch D): E-stop ======
static void task_safety(void *arg)
{
    for (;;) {
        if (xSemaphoreTake(semEStop, portMAX_DELAY) == pdTRUE) {
            int64_t t0 = esp_timer_get_time();

            // Ação crítica: zera "PWM", trava atuadores, sinaliza alarme (~0.9 ms)
            g_belt.set_rpm = 0.f;
            cpu_tight_loop_us(900);

            int64_t dt = esp_timer_get_time() - t0;
        }
    }
}



// ====== app_main ======
void app_main(void)
{
    
    // Cria tasks (afinidade no core 0 para reduzir jitter, opcional)
    xTaskCreatePinnedToCore(task_safety,   "SAFETY",   STK, NULL, PRIO_ESTOP, &hSAFE, 0);
    xTaskCreatePinnedToCore(task_enc_sense,"ENC_SENSE",STK, NULL, PRIO_ENC,   &hENC,  0);
    xTaskCreatePinnedToCore(task_spd_ctrl, "SPD_CTRL", STK, NULL, PRIO_CTRL,  &hCTRL, 0);
    xTaskCreatePinnedToCore(task_sort_act, "SORT_ACT", STK, NULL, PRIO_SORT,  &hSORT, 0);

}
