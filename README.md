## Periféricos configurados en STM32CubeIDE

El microcontrolador utilizado es el **STM32F103RCTx (LQFP64)**.
A continuación se describen los periféricos activados y su rol
dentro del sistema de control del robot balanceador tipo péndulo
invertido sobre ruedas (PIR).

---

## Variables de estado

El modelo dinámico del robot se describe con 6 variables de estado:

| Variable | Descripción |
|---|---|
| x | Desplazamiento horizontal del chasis [m] |
| v | Velocidad lineal del chasis [m/s] |
| θc | Ángulo de inclinación respecto a la vertical [rad] |
| ωc | Velocidad angular de inclinación [rad/s] |
| δ | Ángulo de giro del robot [rad] |
| ωδ | Velocidad angular de giro [rad/s] |

---

## Periféricos y su relación con las variables de estado

### I2C1 — Sensor MPU6050
Comunicación con el sensor MPU6050 (giroscopio + acelerómetro de 6 ejes).
- **SCL** → PB8
- **SDA** → PB9
- **Variables que alimenta:** θc, ωc
- Provee el ángulo de inclinación y la velocidad angular,
  fundamentales para mantener el balance del robot.

### TIM1 — Encoder (motor izquierdo)
Configurado en **Encoder Mode** (CH1: PA8, CH2: PA9).
- Prescaler: 0 | Counter Period: 65535
- **Variable que alimenta:** vi (velocidad tangencial rueda izquierda)

### TIM2 — Encoder (motor derecho)
Configurado en **Encoder Mode** (CH1: PA0, CH2: PA1).
- Prescaler: 0 | Counter Period: 65535
- **Variable que alimenta:** vd (velocidad tangencial rueda derecha)
- A partir de vi y vd se calculan:
  - v = (vi + vd) / 2 → velocidad lineal
  - ωδ = (vd - vi) / d → velocidad angular de giro

### TIM4 — PWM (señales de control a los motores)
Configurado en **PWM Generation** (CH1: PB6, CH2: PB7).
- Prescaler: 0 | Counter Period: 399 → frecuencia PWM: **20 kHz**
- **Variable que controla:** τi y τd (torques de cada motor)
- El duty cycle entre 0 y 399 determina la potencia enviada a cada motor.

### TIM6 — Loop de control (200 Hz)
Timer básico con interrupción cada **5 ms**.
- Prescaler: 79 | Counter Period: 499
- En su callback se ejecuta:
  1. Lectura del MPU6050 → actualiza θc, ωc
  2. Lectura de encoders → actualiza v, ωδ, x (integrado)
  3. Cálculo del controlador (PID / LQR)
  4. Actualización del PWM → aplica τi, τd

### USART — Comunicación con ESP32
Puerto serie para comunicación con la placa ESP32,
encargada de la comunicación microROS con el sistema ROS2.
- Modo: Asíncrono | Baudrate: 115200
- Transmite datos de estado del robot hacia ROS2
- Recibe comandos de velocidad (v, ωδ) desde ROS2
