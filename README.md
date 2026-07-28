# Control de un Robot de Balance (MicroROS Self-Balancing Robot Car)

El presente repositorio consta de la documentación para el proyecto de la asignatura de Control, el cual consistió en diseñar dos controladores —un LQR y un PID— para un robot de balance tipo *MicroROS Self-Balancing Robot Car*. A continuación se presenta la explicación de los controladores implementados.

## PID

### Matrices del Sistema (Continuo)

**Estado:** `x = [posición, vel. lineal, ángulo, vel. angular, ángulo giro, vel. giro]`

```julia
Q_aux = J_centroide * M + (J_centroide + M*L^2) * (2*m + 2*inercia/r^2)

A_23 = -(M^2 * L^2 * g) / Q_aux
A_43 =  M * L * g * (M + 2*m + 2*inercia/r^2) / Q_aux

B_21 = (J_centroide + M*L^2 + M*L*r) / (Q_aux * r)
B_22 = B_21

B_41 = -(M*L/r + M + 2*m + 2*inercia/r^2) / Q_aux
B_42 = B_41

B_61 =  1 / (r * (m*d + inercia*d/r^2 + 2*J_Y_delta/d))
B_62 = -B_61

# ============================================================
# 3. MATRICES DEL SISTEMA (CONTINUO) Y DISCRETIZACIÓN
#    Estado: x = [posición, vel. lineal, ángulo, vel. angular,
#                 ángulo giro, vel. giro]
# ============================================================

A = [0  1    0    0  0  0;
     0  0  A_23   0  0  0;
     0  0    0    1  0  0;
     0  0  A_43   0  0  0;
     0  0    0    0  0  1;
     0  0    0    0  0  0]

B = (inercia/r) .* [0     0   ;
                    B_21  B_22;
                    0     0   ;
                    B_41  B_42;
                    0     0   ;
                    B_61  B_62]
```

A partir de este modelo se diseñaron tres controladores: dos PD (uno para la posición y otro para el ángulo de giro) y un PI para la velocidad angular.

### PD (Posición)

Dado que en este caso no se buscaba controlar el ángulo de giro, el sistema se redujo a 4 variables de estado, combinando ambas señales de control en una sola entrada:

```julia
Ai = A[1:4, 1:4]
Bi = B[1:4,1] + B[1:4,2]
Ci = [0.0  0.0  1.0  0.0]
Di = [0.0]
```

La función de transferencia en el dominio del tiempo continuo resultante es:

$$G(s) = \frac{-2.8026801158626995}{1.0s^2 - 410.9716109240051}$$

Al tratarse de un sistema naturalmente inestable —comportamiento esperable dado que el modelo se asemeja al de un péndulo invertido—, sus polos se ubican a ambos lados del eje imaginario:

```
Polos de G: [-20.27243475569733, 20.272434755697333]
```

Para el diseño del controlador se empleó el método de *Loop Shaping*, implementado en Julia. Se fijaron los siguientes parámetros de diseño:

```julia
ωgc = 220     # Frecuencia de cruce de ganancia [rad/s]
ϕm  = 40.0    # Margen de fase deseado [°]
```

Los valores de ganancia obtenidos fueron:

```julia
Kp = -13341.291913464405
Kd = -50.88487783643825
```

Las ganancias resultan negativas debido a que la planta misma tiene ganancia negativa; este signo es necesario para preservar la realimentación negativa del lazo.

Con estas ganancias, la función de transferencia del controlador (`Kp + Kd*s`) es:

```
      -50.88487783643825s - 13341.291913464405
      -------------------------------------------
                        1.0
```

Con el controlador definido, se construyó la función de transferencia de lazo cerrado:

```
Lazo cerrado: TransferFunction{Continuous, ControlSystemsBase.SisoRational{Float64}}

     142.61403531028807s + 37391.37356578652
-------------------------------------------------
     1.0s^2 + 142.61403531028807s + 36980.401954862515
```

### PD (Ángulo de Giro)

Se realizó un proceso muy similar al caso anterior; sin embargo, en esta ocasión no se llevó a cabo la reducción del sistema, sino que se trabajó directamente con el modelo completo de seis variables de estado. Al hacerlo, se obtuvo una matriz de funciones de transferencia, dado que ahora se tienen dos señales de control y seis variables de estado.

Como el interés recae únicamente en la variable de estado 5 (asociada al ángulo de giro), se tomó la función de transferencia correspondiente a dicha variable de estado con respecto a la señal de control 1:

```
1.0124987969622907
------------------
      1.0s^2
```

Nuevamente, mediante el método de *Loop Shaping*, se fijaron los siguientes parámetros de diseño:

```julia
ωgc = 40      # Frecuencia de cruce de ganancia [rad/s]
ϕm  = 25.0    # Margen de fase deseado [°]
```

Los valores de ganancia obtenidos fueron:

```julia
Kp = 1432.191785
Kd = 16.696050
```

Con estas ganancias se armó el controlador (`Kp + Kd*s`):

```
16.696049931462355s + 1432.191784927767
---------------------------------------
                  1.0
```

Y la función de transferencia de lazo cerrado resultante es:

```
    16.90473046962797s + 1450.09245925864
----------------------------------------------
1.0s^2 + 16.90473046962797s + 1450.092459258
```
