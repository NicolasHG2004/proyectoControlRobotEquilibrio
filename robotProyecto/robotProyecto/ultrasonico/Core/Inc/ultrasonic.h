#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_

#include "main.h"

/* ---------------------------------------------------------------
 * Pines (deben coincidir con lo configurado en CubeMX):
 *   TRIG → PA0  (GPIO_Output)
 *   ECHO → PA1  (TIM2_CH2, Input Capture)
 * --------------------------------------------------------------- */
#define TRIG_PIN    GPIO_PIN_0
#define TRIG_PORT   GPIOA

/* Handle del TIM2 generado por CubeIDE (definido en main.c) */
extern TIM_HandleTypeDef htim2;

/* Función principal: devuelve distancia en cm */
uint32_t Ultrasonic_GetDistance_cm(void);

/* Callback de captura — llámalo desde HAL_TIM_IC_CaptureCallback en main.c */
void Ultrasonic_CaptureCallback(TIM_HandleTypeDef *htim);

/* Callback de overflow — llámalo desde HAL_TIM_PeriodElapsedCallback en main.c */
void Ultrasonic_OverflowCallback(TIM_HandleTypeDef *htim);

#endif /* INC_ULTRASONIC_H_ */
