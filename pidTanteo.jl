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
Ci = [1.0  0.0  0.0  0.0]
Di = [0.0]

# Sistema en espacio de estados (tiempo continuo)
sys_c = ss(Ai, Bi, Ci, Di)
sys_c = minreal(sys_c,fast=false, balance=true)

Ts = 0.01

G = tf(sys_c)
println("La función de transferencia Continua SISO es ", G)

sys_d = c2d(sys_c, Ts, :zoh)

Ad, Bd, Cd, Dd = sys_d.A, sys_d.B, sys_d.C, sys_d.D

sys = ss(Ad, Bd, Cd, Dd, Ts)

G_d = tf(sys)

SP = 5

zeta = abs(log(SP / 100)) / sqrt(log(SP / 100)^2 + π^2)

tr = 0.9 # s

wn = (2.23 * zeta^2 + 0.036 * zeta + 1.54) / tr  # rad/s
println("wn = ", wn)
println("zeta = ", zeta)
println("SP = ", SP)
println("tr = ", tr)


p_rlocus = rlocusplot(
    G,
    title = "Lugar de las raíces",
    xlabel = "Parte Real",
    ylabel = "Parte Imaginaria",
    legend = false,
    grid = true,
    size = (800,600)
)

p_rlocus = rlocusplot(
    G,
    title = "Lugar de las raíces",
    xlabel = "Parte Real",
    ylabel = "Parte Imaginaria",
    legend = false,
    grid = true,
    size = (800,600)
)

display(p_rlocus)

# ============================================================
# CONTROLADOR PID CONTINUO
# ============================================================

s = tf("s")

# Ganancias iniciales (ajustar manualmente)
Kp = 3000
Kd = 10
Ki = 300
N = 20

C = Kp + Ki/s + Kd*(N*s)/(s + N)

# Lazo abierto
L = minreal(C*G)

# Lazo cerrado
T = feedback(L)

# ============================================================
# RESPUESTA AL ESCALÓN
# ============================================================

Tf = 20.0
t = 0:0.001:Tf

p = plot(
    step(T, t),
    title = "Respuesta al escalón",
    xlabel = "Tiempo [s]",
    ylabel = "Salida",
    lw = 2,
    label = "Salida"
)

Cd = c2d(C, Ts, :tustin)

# ============================================================
# Lazo cerrado discreto
# ============================================================

Ld = Cd * G_d
Td = feedback(Ld)


polosC = ControlSystems.poles(T)
polosD = ControlSystems.poles(Td)

println("Polos continuo: ", polosC)
println("Polos discreto: ", polosD)
    # ============================================================
# Respuesta al escalón
# ============================================================

Tf = 3.0
t = 0:Ts:Tf

p1 = plot(
    step(Td, t),
    title = "Respuesta al escalón (Discreto)",
    xlabel = "Tiempo [s]",
    ylabel = "Salida",
    lw = 2,
    label = "Salida"
)


# ============================================================
# Señal de control
# ============================================================

r = ones(1, length(t))

# Salida del sistema
y, _, _ = lsim(Td, r, t)

# Error
e = r .- y

# Señal de control
u, _, _ = lsim(Cd, e, t)

p2 = plot(
    t,
    vec(u),
    title = "Señal de control",
    xlabel = "Tiempo [s]",
    ylabel = "u[k]",
    lw = 2,
    label = "u"
)


println("u_max = ", maximum(u))
println("u_min = ", minimum(u))

# ============================================================
# FIGURA FINAL
# ============================================================

p_cont = plot(
    step(T, t),
    title = "Respuesta al escalón (Continuo)",
    xlabel = "Tiempo [s]",
    ylabel = "Salida",
    lw = 2,
    label = "Continuo"
)

p_disc = plot(
    step(Td, t),
    title = "Respuesta al escalón (Discreto)",
    xlabel = "Tiempo [s]",
    ylabel = "Salida",
    lw = 2,
    label = "Discreto"
)

p_control = plot(
    t,
    vec(u),
    title = "Señal de control",
    xlabel = "Tiempo [s]",
    ylabel = "u[k]",
    lw = 2,
    label = "Control"
)

plot(
    p_cont,
    p_disc,
    p_control,
    layout = (3,1),   # 3 filas, 1 columna
    size = (900,900)
)
