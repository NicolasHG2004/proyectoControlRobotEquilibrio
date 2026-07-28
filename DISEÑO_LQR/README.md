# Proceso iterativo de sintonización del controlador LQR

A continuación se describe el proceso iterativo seguido para la sintonización del controlador LQR, indicando las ganancias utilizadas en cada prueba y las observaciones obtenidas.

---

## Iteración 1

**Ganancias del controlador:**

- K1 = -99.99
- K2 = -89.39
- K3 = -344.54
- K4 = -21.83
- K5 = 99.99
- K6 = 29.97

**Observaciones:**

- La estabilización del robot presenta menores oscilaciones.
- Sin embargo, el giro continúa siendo inestable.

---

## Iteración 2

**Ganancias del controlador:**

- K1 = -83.66
- K2 = -80.80
- K3 = -339.80
- K4 = -21.35
- K5 = 99.99
- K6 = 29.97

**Pesos de la matriz \(Q\):**

| Variable | Peso |
|----------|-----:|
| Posición | 70000 |
| Velocidad lineal | 100 |
| Ángulo de cabeceo | 5000 |
| Velocidad angular | 700 |
| Ángulo de giro | 100000 |
| Velocidad de giro | 8000 |

**Observaciones:**

- La estabilización del robot presenta menores oscilaciones.
- El giro continúa siendo inestable.

---

## Iteración 3

**Ganancias del controlador:**

- K1 = -31.62
- K2 = -47.11
- K3 = -319.86
- K4 = -19.43
- K5 = 31.62
- K6 = 28.83

**Pesos de la matriz \(Q\):**

| Variable | Peso |
|----------|-----:|
| Posición | 10000 |
| Velocidad lineal | 100 |
| Ángulo de cabeceo | 500 |
| Velocidad angular | 700 |
| Ángulo de giro | 10000 |
| Velocidad de giro | 8000 |


**Observaciones:**

Mejorora en el giro sin embargo es muy suceptible a perturbaciones en el ángulo de cabeceo lo que haceque al iniciar unpoco inclinado no sea capaz de estabilizarse o oscile demaiado para hacerlo


## Iteración 4

**Ganancias del controlador:**

- K1 = -8.36
- K2 = -23.90
- K3 = -312.96
- K4 = -18.23
- K5 = 31.62
- K6 = 28.83

**Pesos de la matriz \(Q\):**

| Variable | Peso |
|----------|-----:|
| Posición | 700 |
| Velocidad lineal | 100 |
| Ángulo de cabeceo | 20000 |
| Velocidad angular | 700 |
| Ángulo de giro | 10000 |
| Velocidad de giro | 8000 |


**Observaciones:**

- Mayor robustez pero, sin embargo se nota una disminucipon en el exfuerzo de control

## Iteración 5

**Ganancias del controlador:**

- K1 = -22.36
- K2 = -41.42
- K3 = -334.17
- K4 = -22.24
- K5 = 49.99
- K6 = 45.27

**Pesos de la matriz \(Q\):**

| Variable | Peso |
|----------|-----:|
| Posición | 2000 |
| Velocidad lineal | 100 |
| Ángulo de cabeceo | 20000 |
| Velocidad angular | 700 |
| Ángulo de giro | 10000 |
| Velocidad de giro | 8000 |

En la matriz R se movió de 5 -> 2


**Observaciones:**

- El robot se mueve con maz fuerza y responde más rápido, es mas robusto a perturbaciones.