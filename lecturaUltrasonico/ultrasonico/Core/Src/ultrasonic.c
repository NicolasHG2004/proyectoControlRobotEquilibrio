#include "ultrasonic.h"

/*
 * Lógica idéntica al ejemplo Keil, reescrita con HAL de CubeIDE.
 *
 * El TIM2 corre a 1 MHz (1 tick = 1 µs).
 * Eso se logra con Prescaler = 71 y ARR = 65535
 * (para un STM32F103 a 72 MHz).
 *
 * Flujo:
 *  1. Se envía pulso TRIG de 15 µs.
 *  2. El TIM2_CH2 captura el flanco de subida de ECHO → guarda tiempo inicio.
 *  3. Se reconfigura para capturar flanco de bajada.
 *  4. Captura el flanco de bajada → calcula duración.
 *  5. distancia (cm) = duración_us * 0.017  (velocidad sonido / 2)
 */

/* Estados internos */
static volatile uint8_t  capture_state = 0;  /* 0=esperando subida, 1=esperando bajada */
static volatile uint32_t capture_start = 0;
static volatile uint32_t overflow_count = 0;
static volatile uint32_t pulse_duration_us = 0;
static volatile uint8_t  capture_done = 0;

/* Pequeño delay en microsegundos usando el propio TIM2 como referencia.
 * Se usa solo para el pulso TRIG; TIM2 debe estar corriendo. */
static void delay_us(uint32_t us)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while ((__HAL_TIM_GET_COUNTER(&htim2) - start) < us);
}

/* ---------------------------------------------------------------
 * Ultrasonic_GetDistance_cm
 *   Envía el pulso TRIG, espera la captura del ECHO y retorna
 *   la distancia en centímetros. Timeout ~60 ms.
 * --------------------------------------------------------------- */
uint32_t Ultrasonic_GetDistance_cm(void)
{
    /* Reiniciar estado */
    capture_done     = 0;
    capture_state    = 0;
    overflow_count   = 0;
    pulse_duration_us = 0;

    /* Configurar canal para capturar flanco de SUBIDA */
    TIM_IC_InitTypeDef ic_cfg = {0};
    ic_cfg.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    ic_cfg.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic_cfg.ICPrescaler = TIM_ICPSC_DIV1;
    ic_cfg.ICFilter    = 0;
    HAL_TIM_IC_ConfigChannel(&htim2, &ic_cfg, TIM_CHANNEL_2);

    /* Activar captura + interrupción de overflow */
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

    /* Pulso TRIG: mínimo 10 µs, usamos 15 µs */
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_us(15);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    /* Esperar captura completa (timeout 60 ms) */
    uint32_t t_start = HAL_GetTick();
    while (!capture_done)
    {
        if ((HAL_GetTick() - t_start) > 60)
        {
            /* Timeout: no hay eco, detener y retornar 0 */
            HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
            __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
            return 0;
        }
    }

    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);

    /* distancia (cm) = tiempo_us * velocidad_sonido / 2
     * velocidad sonido ≈ 34000 cm/s → 0.034 cm/µs → /2 = 0.017 cm/µs
     * Para evitar float: distancia = tiempo_us * 17 / 1000  */
    return (pulse_duration_us * 17UL) / 100UL;
}

/* ---------------------------------------------------------------
 * Ultrasonic_CaptureCallback
 *   Llamar desde HAL_TIM_IC_CaptureCallback en main.c
 * --------------------------------------------------------------- */
void Ultrasonic_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;
    if (htim->Channel  != HAL_TIM_ACTIVE_CHANNEL_2) return;

    if (capture_state == 0)
    {
        /* Primera captura: flanco de SUBIDA */
        capture_start  = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        overflow_count = 0;
        capture_state  = 1;

        /* Reconfigurar para capturar flanco de BAJADA */
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
                                      TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else
    {
        /* Segunda captura: flanco de BAJADA */
        uint32_t capture_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

        /* Calcular duración teniendo en cuenta overflows del contador (ARR=65535) */
        if (capture_end >= capture_start)
        {
            pulse_duration_us = capture_end - capture_start
                                + overflow_count * 65536UL;
        }
        else
        {
            /* El contador dio la vuelta entre subida y bajada */
            pulse_duration_us = (65535UL - capture_start) + capture_end + 1UL
                                + overflow_count * 65536UL;
        }

        capture_state = 0;
        capture_done  = 1;
    }
}

/* ---------------------------------------------------------------
 * Ultrasonic_OverflowCallback
 *   Llamar desde HAL_TIM_PeriodElapsedCallback en main.c
 * --------------------------------------------------------------- */
void Ultrasonic_OverflowCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;

    if (capture_state == 1)   /* Solo contar si estamos midiendo */
        overflow_count++;
}
