# ============================================================
# DISEÑO PD POR LOOP SHAPING
# ============================================================

# ============================================================
#                   MODELO LQR DISCRETO
# ============================================================

using LinearAlgebra
using Printf
using ControlSystems
using Plots
using LaTeXStrings
using Polynomials

# ============================================================
# 1. PARÁMETROS FÍSICOS DEL SISTEMA
# ============================================================

m       = 0.035
r       = 0.0672 / 2
inercia = 0.5 * m * r^2

M = 1.000 - 2*m
L = 0.5 * 0.0766

J_centroide = (1/12) * M * (0.0766^2 + 0.0575^2)
d           = 0.1612
J_Y_delta   = (1/12) * M * (0.0766^2 + 0.0575^2)
g           = 9.8


# ============================================================
# 2. TÉRMINOS AUXILIARES PARA LAS MATRICES A Y B
# ============================================================

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

Ai= A[1:4, 1:4]
Bi = B[1:4,1] + B[1:4,2]
Ci = [0.0  0.0  1.0  0.0]
Di = [0.0]

# Sistema en espacio de estados (tiempo continuo)
sys_c = ss(Ai, Bi, Ci, Di)
sys_c = minreal(sys_c,fast=false, balance=true)

Ts = 0.01

G = tf(sys_c)

# s = tf("s")

# G = 5/(2s + 1)

println("La función de transferencia Continua SISO es ", G)

# ============================================================
# 4. DISEÑO PD POR LOOP SHAPING
# ============================================================

ωgc = 220          # Frecuencia de cruce de ganancia [rad/s]
ϕm  = 40.0          # Margen de fase deseado [°]

C, kp, kd, fig, CF = loopshapingPD(
    G,
    rl=1,
    ωgc;
    phasemargin = ϕm,
    form = :parallel,
    doplot = true
)

println("\n======================================")
println("CONTROLADOR PD DISEÑADO")
println("======================================")

@printf("Kp = %.6f\n", kp)
@printf("Kd = %.6f\n", kd)

println("\nControlador:")
println(C)

# ============================================================
# 5. LAZO ABIERTO Y LAZO CERRADO
# ============================================================

L = C * G
T = feedback(L)

println("\nMárgenes de estabilidad:")

gm, pm, ωcg, ωcp = margin(L)

println("Margen de ganancia = ", gm)
println("Margen de fase = ", pm)
println("Frecuencia de cruce de ganancia = ", ωcg)
println("Frecuencia de cruce de fase = ", ωcp)



println(G)

println("Polos de G:")
polosG =ControlSystems.poles(G)
println("Polos de G ", polosG)

println("C:")
println(C)

println("Polos de T:")
polosT =ControlSystems.poles(T)
println("Polos de T ", polosT)

# ============================================================
# 6. GRÁFICAS
# ============================================================

# display(bodeplot(L,
#     title = "Bode del lazo abierto"))


p = plot(impulse(T),
    title = "Respuesta al Impulso del sistema en lazo cerrado",
    xlabel = "Tiempo [s]",
    ylabel = "Salida"
)

display(p)

println("Kp = ", kp)
println("Kd = ", kd)

println("Lazo cerrado", T)
marginplot(L)

# display(pzmap(T))