## Periféricos configurados en STM32CubeIDE

El microcontrolador utilizado es el **STM32F103RCTx (LQFP64)**.
A continuación se describen los periféricos activados y su rol
dentro del sistema de control del robot balanceador.

### I2C1
Comunicación con el sensor **MPU6050** (giroscopio + acelerómetro
de 6 ejes). Provee el ángulo de inclinación θc y la velocidad
angular ωc, variables fundamentales para el control de balance.

### TIM1 — Encoder (motor izquierdo)
Configurado en **Encoder Mode** (CH1 y CH2).
- Prescaler: 0 | Counter Period: 65535
- Permite medir la velocidad tangencial vi de la rueda izquierda.

### TIM2 — Encoder (motor derecho)
Configurado en **Encoder Mode** (CH1 y CH2).
- Prescaler: 0 | Counter Period: 65535
- Permite medir la velocidad tangencial vd de la rueda derecha.
- Con vi y vd se calculan v (velocidad lineal) y ωδ (velocidad angular de giro).

### TIM3 — PWM (señales de control a los motores)
- **CH3** → PWM Generation: señal de potencia motor izquierdo
- **CH4** → PWM Generation: señal de potencia motor derecho
- Prescaler: 0 | Counter Period: 399 → frecuencia PWM: **20 kHz**

### TIM6 — Loop de control (200 Hz)
Timer básico con interrupción cada **5 ms**.
- Prescaler: 79 | Counter Period: 499
- En su callback se ejecuta la lectura de sensores,
  el cálculo del controlador y la actualización del PWM.

### USART
Puerto serie para comunicación con la placa **ESP32**,
encargada de la comunicación microROS con el sistema ROS2.
- Modo: Asíncrono | Baudrate: 115200
