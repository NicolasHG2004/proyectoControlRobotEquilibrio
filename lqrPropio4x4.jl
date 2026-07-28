# ============================================================
#           MODELO LQR DISCRETO — SISTEMA REDUCIDO (4 ESTADOS)
#   Estado: x = [posición, vel. lineal, ángulo θ, vel. angular θ̇]
#   Se elimina el eje de giro lateral ψ (x5, x6) y se colapsan
#   los dos inputs de las ruedas en un solo input efectivo.
# ============================================================

using LinearAlgebra
using Printf
using ControlSystems
using Plots
using LaTeXStrings


# ============================================================
# 1. PARÁMETROS FÍSICOS DEL SISTEMA
# ============================================================

m       = 0.035
r       = 0.0672 / 2
inercia = 0.5 * m * r^2

M = 1.000 - 2*m
L = 0.5 * 0.0766

J_centroide = (1/12) * M * (0.0766^2 + 0.0575^2)
g           = 9.8


# ============================================================
# 2. TÉRMINOS AUXILIARES PARA LAS MATRICES A Y B (4 estados)
# ============================================================

Q_aux = J_centroide * M + (J_centroide + M*L^2) * (2*m + 2*inercia/r^2)

A_23 = -(M^2 * L^2 * g) / Q_aux
A_43 =  M * L * g * (M + 2*m + 2*inercia/r^2) / Q_aux

B_21 = (J_centroide + M*L^2 + M*L*r) / (Q_aux * r)
B_22 = B_21   # los dos motores tienen el mismo efecto sobre este subsistema

B_41 = -(M*L/r + M + 2*m + 2*inercia/r^2) / Q_aux
B_42 = B_41


# ============================================================
# 3. MATRICES DEL SISTEMA REDUCIDO (CONTINUO) Y DISCRETIZACIÓN
# ============================================================

Ai = [0  1    0    0;
      0  0  A_23   0;
      0  0    0    1;
      0  0  A_43   0]

# Bi: suma de los dos inputs (B_21+B_22, B_41+B_42), ya que ambos
# motores afectan igual al subsistema posición-ángulo
Bi = (inercia/r) .* [0.0; B_21 + B_22; 0.0; B_41 + B_42]

# Salida "solo ángulo", útil para sacar la función de transferencia
Ctheta = [0.0  0.0  1.0  0.0]
Dtheta = [0.0]

sys_c    = ss(Ai, Bi, Ctheta, Dtheta)
sys_c    = minreal(sys_c, fast=false, balance=true)
tfangulo = tf(sys_c)

Ts = 0.01

# Salida "estado completo" (para diseño de LQR/Kalman con medición
# directa y ruidosa de las 4 variables)
C_full = Matrix{Float64}(I, 4, 4)
D_full = zeros(4, 1)

sys_c_full = ss(Ai, Bi, C_full, D_full)
sys_d      = c2d(sys_c_full, Ts, :zoh)

Ad, Bd, Cd, Dd = sys_d.A, sys_d.B, sys_d.C, sys_d.D
sys = ss(Ad, Bd, Cd, Dd, Ts)


# ============================================================
# 4. CONTROLABILIDAD
# ============================================================

Wr      = ctrb(sys)
rank_Wr = rank(Wr)
println("Rango de controlabilidad: ", rank_Wr, " / ", size(Ad, 1))


# ============================================================
# 5. DISEÑO DEL LQR DISCRETO
# ============================================================

Q_lqr = diagm([
7700.0,
0.0,
0.0,
1600.0
])

R_lqr = [1.0;;]

K_d = lqr(sys, Q_lqr, R_lqr)

println("\nMatriz de ganancias K discreta:")
display(K_d)

Acl_d = Ad - Bd * K_d   # Matriz de lazo cerrado DISCRETA

polos_cl = eigvals(Acl_d)
println("\nPolos del lazo cerrado discreto ")
display(polos_cl)


# ============================================================
# 6. FILTRO DE KALMAN DISCRETO
# ============================================================

# Perturbación asociada a la deducción mala del modelo (proceso)
Rw = diagm([
0.001,
0.01,
0.001,
0.01
])

# Ruido de sensores: posición, velocidad lineal, ángulo, vel. angular
Rv = diagm([0.01, 0.02, 0.005, 0.005])

Rw_d = Rw .* Ts
Rv_d = Rv

L_d = kalman(sys, Rw_d, Rv_d)

println("Imprimo la dimensión de L_d")
println(typeof(L_d))
display(L_d)

# Como Cd = I, la dinámica del error de estimación es (Ad - L_d*Cd)
polos_est = eigvals(Ad - L_d*Cd)
println("¿Estimador estable? ", all(abs.(polos_est) .< 1))
println("¿Lazo LQG completo estable? ",
        all(abs.(polos_cl) .< 1) && all(abs.(polos_est) .< 1))


# ============================================================
# 7. PRECOMPENSADOR (seguimiento de posición, x1)
# ============================================================

C_pos = [1.0  0.0  0.0  0.0]   # Salida: x1 (posición)

DC_pos_d = (C_pos * inv(I - Acl_d) * Bd)[1]
kr_d     = 1.0 / DC_pos_d

@printf "\nPrecompensador kr discreto (seguimiento posición) = %.6f\n" kr_d

Bd_r  = Bd .* kr_d                     # Vector de entrada efectivo (4x1)
C_all = Matrix{Float64}(I, 4, 4)
D_r   = zeros(4, 1)

sys_lc_d = ss(Acl_d, Bd_r, C_all, D_r, Ts)   # Sistema discreto para lsim


# ============================================================
# 8. SIMULACIÓN DEL LQG COMPLETO (CON FILTRO DE KALMAN)
# ============================================================

t_sim = 0.0:Ts:5.0
ref   = ones(length(t_sim))

function simular_lqg()
    N = length(t_sim)

    X       = zeros(4, N)
    X_gorro = zeros(4, N)
    Y       = zeros(4, N)
    U       = zeros(1, N)

    x       = [0.1, 0, 0, 0]
    x_gorro = zeros(4)

    X[:, 1]       = x
    X_gorro[:, 1] = x_gorro
    Y[:, 1]       = Cd * x

    for k = 1:N-1
        r = ref[k]
        u = -K_d * x_gorro .+ kr_d * r          # 1 elemento (single input)

        x  = Ad*x + Bd*u
        vk = randn(4) .* 0.01                    # ruido de medición (nuevo cada paso)
        y  = Cd*x + vk

        x_gorro = Ad*x_gorro + Bd*u + L_d*(y - Cd*x_gorro)

        X[:, k+1]       = x
        X_gorro[:, k+1] = x_gorro
        Y[:, k+1]       = y
        U[:, k]         = u
    end

    return X, X_gorro, Y, U
end

X, X_gorro, Y, U = simular_lqg()


# ============================================================
# 9. GRÁFICAS
# ============================================================

# --- Ángulo de inclinación ---
p1 = plot(t_sim, X[3, :],
    label="θ real",
    linewidth=2)

plot!(p1, t_sim, X_gorro[3, :],
    label="θ estimado",
    linewidth=2,
    linestyle=:dash)

xlabel!(p1, "Tiempo [s]")
ylabel!(p1, "Ángulo [rad]")
title!(p1, "Estimación de θ")


# --- Posición ---
p2 = plot(t_sim, X[1, :],
    label="Posición real")

plot!(p2, t_sim, X_gorro[1, :],
    label="Posición estimada",
    linestyle=:dash)

plot!(p2, t_sim, ref,
    label="Referencia",
    linestyle=:dot)

xlabel!(p2, "Tiempo [s]")
ylabel!(p2, "Posición [m]")
title!(p2, "Estimación y seguimiento de posición")


# --- Velocidad angular ---
p3 = plot(t_sim, X[4, :],
    label="θ̇ real")

plot!(p3, t_sim, X_gorro[4, :],
    label="θ̇ estimado",
    linestyle=:dash)

xlabel!(p3, "Tiempo [s]")
ylabel!(p3, "θ̇ [rad/s]")
title!(p3, "Estimación de θ̇")


# --- Error de estimación ---
errorEstimacion = X - X_gorro

p4 = plot(t_sim, errorEstimacion[3, :],
    label="Error θ",
    linewidth=2)

xlabel!(p4, "Tiempo [s]")
ylabel!(p4, "Error [rad]")
title!(p4, "Error de estimación")


fig = plot(p1, p2, p3, p4,
           layout=(2,2),
           size=(1080,720),
           legend=:topright)

display(fig)

