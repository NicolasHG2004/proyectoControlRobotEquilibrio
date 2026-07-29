# ============================================================
# DISEÑO PI POR ASIGNACIÓN DE POLOS (MÉTODO ALGEBRAICO)
# ============================================================

# ============================================================
#    CONTROLADOR DE VELOCIDAD LINEAL (OUTER LOOP)
# ============================================================

using LinearAlgebra
using Printf
using ControlSystems
using Plots
using Polynomials

# ============================================================
# 1. PARÁMETROS FÍSICOS DEL SISTEMA
# ============================================================

m = 0.035
r = 0.0672 / 2
inercia = 0.5 * m * r^2

M = 1.000 - 2 * m
L = 0.5 * 0.0766

J_centroide = (1 / 12) * M * (0.0766^2 + 0.0575^2)
d = 0.1612
J_Y_delta = (1 / 12) * M * (0.0766^2 + 0.0575^2)
g = 9.81


# ============================================================
# 2. TÉRMINOS AUXILIARES PARA LAS MATRICES A Y B
# ============================================================

Q_aux = J_centroide * M + (J_centroide + M * L^2) * (2 * m + 2 * inercia / r^2)

A_23 = -(M^2 * L^2 * g) / Q_aux
A_43 = M * L * g * (M + 2 * m + 2 * inercia / r^2) / Q_aux

B_21 = (J_centroide + M * L^2 + M * L * r) / (Q_aux * r)
B_22 = B_21

B_41 = -(M * L / r + M + 2 * m + 2 * inercia / r^2) / Q_aux
B_42 = B_41

B_61 = 1 / (r * (m * d + inercia * d / r^2 + 2 * J_Y_delta / d))
B_62 = -B_61


# ============================================================
# 3. MATRICES DEL SISTEMA (CONTINUO)
#    Estado: x = [posición, vel. lineal, ángulo, vel. angular,
#                 ángulo giro, vel. giro]
# ============================================================

A = [0 1 0 0 0 0;
    0 0 A_23 0 0 0;
    0 0 0 1 0 0;
    0 0 A_43 0 0 0;
    0 0 0 0 0 1;
    0 0 0 0 0 0]

B = (inercia / r) .* [0 0;
    B_21 B_22;
    0 0;
    B_41 B_42;
    0 0;
    B_61 B_62]

C = Matrix{Float64}(I, 6, 6)
D = zeros(6, 2)

# Sistema en espacio de estados (tiempo continuo)
sys_c = ss(A, B, C, D)

G = tf(sys_c)

# Extraer subsistema de posición/ángulo (estados 1-4, entrada combinada u1+u2)
# Igual que en pdPosicionLS.jl
Ai = A[1:4, 1:4]
Bi = B[1:4, 1] + B[1:4, 2]
Ci = [0.0 0.0 1.0 0.0]   # Salida: ángulo (estado 3)
Di = [0.0]

sys_angulo = ss(Ai, Bi, Ci, Di)
G_angulo = tf(minreal(sys_angulo, fast=false, balance=true))


# ============================================================
# 4. LAZO INTERNO: PD DE BALANCE (RESULTADOS)
# ============================================================
# El lazo interno (Balance_PD) debe diseñarse PRIMERO porque estabiliza
# el ángulo de inclinación del robot. Sin este controlador el sistema es
# inestable y no tiene sentido diseñar el lazo de velocidad.
#
# Los resultados del PD provienen de pdPosicionLS.jl (Loop Shaping).
# Balance_Kp y Balance_Kd son las ganancias continuas de ese diseño.

Balance_Kp = -13341.291913464405
Balance_Kd = -50.88487783643825

C_pd = tf([Balance_Kd, Balance_Kp], [1.0])

# Cerrar el lazo interno: T_inner = feedback(C_pd * G_angulo)
L_inner = C_pd * G_angulo
T_inner = feedback(L_inner)


# ============================================================
# 5. PLANTA SISO DEL LAZO EXTERNO (VELOCIDAD LINEAL)
# ============================================================
# Con el lazo interno cerrado, la planta que ve el PI de velocidad es
# la relación entre la referencia de ángulo y la velocidad lineal.
#
# En estado estacionario del lazo interno:
#   θ_deg = -Velocity_Pwm / Balance_Kp
#   ẍ ≈ g * θ_rad = g * θ_deg * (π/180)
#
# Integrando: G_vel(s) = K_v / s
# donde K_v = -g / Balance_Kp (dado que Balance_Kp está en radianes)

K_v = -g / Balance_Kp

G_vel = tf([K_v], [1.0, 0.0])
G_vel = minreal(G_vel)
println("G_vel = ", G_vel)

# ============================================================
# 6. DISEÑO PI POR ASIGNACIÓN DE POLOS
# ============================================================

zeta = 0.898
wn = 1.7965

polos_deseados = [-zeta * wn + wn * sqrt(Complex(zeta^2 - 1)), -zeta * wn - wn * sqrt(Complex(zeta^2 - 1))]

# Planta G_vel(s) = N(s)/D(s)
num_poly = G_vel.matrix[1].num
den_poly = G_vel.matrix[1].den
N_coeffs = reverse(coeffs(num_poly))
D_coeffs = reverse(coeffs(den_poly))

# Polinomio deseado F(s)
DT = fromroots(polos_deseados)
F_coeffs = reverse(coeffs(DT))

# Ecuación: S(s)D(s) + R(s)N(s) = F(s)
# S(s) = s, R(s) = B1*s + B0
A_mat = [N_coeffs[1] 0.0;
    0.0 N_coeffs[1]]

b_vec = real.([F_coeffs[2], F_coeffs[3]])

x = A_mat \ b_vec
B1, B0 = x[1], x[2]

kp = B1
ki = B0

C = tf([kp, ki], [1.0, 0.0])

println("\n======================================")
println("CONTROLADOR PI DISEÑADO")
println("======================================")
println("Polos deseados: ", polos_deseados)

@printf("Kp = %.6f\n", kp)
@printf("Ki = %.6f\n", ki)

println("\nControlador:")
println(C)

# ============================================================
# 7. LAZO ABIERTO Y LAZO CERRADO
# ============================================================

L = C * G_vel
T = feedback(L)

println("\nMárgenes de estabilidad:")

gm, pm, ωcg, ωcp = margin(L)

println("Margen de ganancia = ", gm)
println("Margen de fase = ", pm)
println("Frecuencia de cruce de ganancia = ", ωcg)
println("Frecuencia de cruce de fase = ", ωcp)

println(G_vel)

println("Polos de G:")
polosG = ControlSystems.poles(G_vel)
println("Polos de G ", polosG)

println("C:")
println(C)

println("Polos de T:")
polosT = ControlSystems.poles(T)
println("Polos de T ", polosT)

# ============================================================
# 8. GRÁFICAS
# ============================================================

mkpath("imagenes")

p_freq = marginplot(L)
savefig(p_freq, "imagenes/piVelocidadFrecuencia.png")

t = 0:0.01:5
p_salida = plot(impulse(T, t),
    title="Respuesta al impulso del sistema en lazo cerrado",
    xlabel="Tiempo [s]",
    ylabel="Velocidad [m/s]"
)
savefig(p_salida, "imagenes/piVelocidadSalida.png")

# ============================================================
# 9. CONVERSIÓN A UNIDADES DEL FABRICANTE (Velocity_PI)
# ============================================================

# 1 m/s = 6271.1 pulsos/s
# El firmware toma lecturas de encoder cada 5ms = 31.35 pulsos por 5ms (por rueda)
# El firmware suma la lectura de AMBAS ruedas (izq + der)
# escala_velocidad = pulsos_por_5ms * 2 = 62.704 pulsos por m/s en cada ciclo
escala_velocidad = 62.70427382215649

# Conversión Kp:
# Velocity_Kp_calc = (Kp_continuo / escala_velocidad) * 100
Velocity_Kp_calc = (abs(kp) / escala_velocidad) * 100

# Conversión Ki:
# El firmware acumula el error a 200 Hz. Integral_firmware = 200 * ∫ error dt
# Velocity_Ki_calc = (Ki_continuo / (escala_velocidad * 200)) * 100
Velocity_Ki_calc = (abs(ki) / (escala_velocidad * 200)) * 100

println("\n======================================")
println("CONSTANTES PARA EL FIRMWARE (Velocity_PI)")
println("======================================")
@printf("Velocity_Kp = %.0f  (Referencia del fabricante: 7000)\n", Velocity_Kp_calc)
@printf("Velocity_Ki = %.0f    (Referencia del fabricante: 35)\n", Velocity_Ki_calc)
