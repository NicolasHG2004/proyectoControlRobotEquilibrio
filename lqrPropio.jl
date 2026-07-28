# ============================================================
#                   MODELO LQR DISCRETO
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

C = Matrix{Float64}(I, 6, 6)
D = zeros(6, 2)

Ts = 0.005

sys_c = ss(A, B, C, D)

# Discretización del sistema
sys_d = c2d(sys_c, Ts, :zoh)

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

Q_lqr = diagm([200000.0, 100.0, 5000.0, 300.0, 50000.0, 3000.0])
R_lqr = [5.0  0.0;
         0.0  5.0]

K= lqr(sys_c, Q_lqr, R_lqr)

println("\n===== Ganancias para C =====")

println(K)

# for i in 1:size(K,1)
#     print("Fila $i: float ")
#     for j in 1:size(K,2)
#         if j < size(K,2)
#             @printf("K = %.6f, ", j, K[i,j])
#         else
#             @printf("K = %.6f;\n", j, K[i,j])
#         end
#     end
# end

Alc = A - B * K   # Matriz de lazo cerrado DISCRETA

polos_cl = eigvals(Alc)
println("\nPolos del lazo cerrado continuo ")
display(polos_cl)


# # ============================================================
# # 6. FILTRO DE KALMAN DISCRETOlqr = 
# # ============================================================
# # Perturbación asociada a la deducción mala del modelo. Es decir,
# # variables y aspectos que desconfío
# Rw = diagm([0.0080, 0.08, 0.001, 0.01, 0.001, 0.01])

# # Este en cambio se asocia con mi desconfianza a los sensores.
# # Seguramente yo confíe más en los de posición y velocidad lineal
# # respecto al de la IMU
# Rv = diagm([
#     0.0001,    # x1: posición [m]     → std ≈ 0.01 m (1 cm)
#     0.0004,    # x2: vel. lineal      → std ≈ 0.02 m/s
#     0.0002,    # x3: ángulo θ [rad]   → std ≈ 0.014 rad (~0.8°)
#     0.001,     # x4: vel. angular     → std ≈ 0.032 rad/s
#     0.0002,    # x5: ángulo ψ [rad]   → std ≈ 0.014 rad
#     0.001      # x6: vel. giro        → std ≈ 0.032 rad/s
# ])
# Rw_d = Rw .* Ts
# Rv_d = Rv

# L_d = kalman(sys, Rw_d, Rv_d)

# println("Imprimo la dimensión de L_d")
# println(typeof(L_d))
# display(L_d)

# polos_est = eigvals(Ad - L_d*Ad)
# println("¿Estimador estable? ", all(abs.(polos_est) .< 1))
# println("¿Lazo LQG completo estable? ",
#         all(abs.(polos_cl) .< 1) && all(abs.(polos_est) .< 1))


# # ============================================================
# # 7. PRECOMPENSADOR (HECHO POR EL FABRICANTE, PASADO A DISCRETO)
# # ============================================================

# C5 = [0.0  0.0  0.0  0.0  1.0  0.0]   # Salida: x5 (ángulo de giro ψ)

# # Esta fórmula se usa en discreto
# DC_dif_d = (C5 * inv(I - Acl_d) * (Bd[:, 1] - Bd[:, 2]))[1]
# kr_d     = 1.0 / DC_dif_d

# @printf "\nPrecompensador kr discreto (seguimiento x5) = %.6f\n" kr_d

# Bd_r  = Bd * [kr_d; -kr_d]             # Vector de entrada efectivo (6x1)
# C_all = Matrix{Float64}(I, 6, 6)
# D_r   = zeros(6, 1)

# sys_lc_d = ss(Acl_d, Bd_r, C_all, D_r, Ts)   # Sistema discreto para lsim


# # ============================================================
# # 8. SIMULACIÓN DEL LQG COMPLETO (CON FILTRO DE KALMAN)
# # ============================================================

# t_sim = 0.0:Ts:10.0
# ref   = ones(length(t_sim))
# Ki = 2.0   # ganancia integral, la sintonizas a mano

# function simular_lqg()
#     N = length(t_sim)
#     X       = zeros(6, N)
#     X_gorro = zeros(6, N)
#     Y       = zeros(6, N)
#     U       = zeros(2, N)

#     x       = [0.1, 0, 0, 0, 0, 0]
#     x_gorro = zeros(6)
#     xi      = 0.0   # acumulador integral

#     X[:, 1]       = x
#     X_gorro[:, 1] = x_gorro
#     Y[:, 1]       = Cd * x

#     for k = 1:N-1
#         r = ref[k]

#         xi = xi + Ts * (x_gorro[1] - 0.0)   # integra el error de posición estimado

#         u = -K_d * x_gorro .+ [kr_d; -kr_d] * r .- Ki * [1.0; -1.0] * xi

#         v = sqrt.(diag(Rv)) .* randn(6)
#         x       = Ad*x + Bd*u
#         y       = Cd*x + v
#         x_gorro = Ad*x_gorro + Bd*u + L_d*(y - Cd*x_gorro)

#         X[:, k+1]       = x
#         X_gorro[:, k+1] = x_gorro
#         Y[:, k+1]       = y
#         U[:, k]         = u
#     end

#     return X, X_gorro, Y, U
# end

# X, X_gorro, Y, U = simular_lqg()


# # ============================================================
# # 9. GRÁFICAS
# # ============================================================

# # --- Ángulo de inclinación ---
# p1 = plot(t_sim, X[3, :],
#     label="θ real",
#     linewidth=2)

# plot!(p1, t_sim, X_gorro[3, :],
#     label="θ estimado",
#     linewidth=2,
#     linestyle=:dash)

# xlabel!(p1, "Tiempo [s]")
# ylabel!(p1, "Ángulo [rad]")
# title!(p1, "Estimación de θ")


# # --- Posición ---
# p2 = plot(t_sim, X[1, :],
#     label="Posición real")

# plot!(p2, t_sim, X_gorro[1, :],
#     label="Posición estimada",
#     linestyle=:dash)

# xlabel!(p2, "Tiempo [s]")
# ylabel!(p2, "Posición [m]")
# title!(p2, "Estimación de la posición")


# # --- Giro lateral ---
# p3 = plot(t_sim, X[5, :],
#     label="ψ real")

# plot!(p3, t_sim, X_gorro[5, :],
#     label="ψ estimado",
#     linestyle=:dash)

# xlabel!(p3, "Tiempo [s]")
# ylabel!(p3, "ψ [rad]")
# title!(p3, "Estimación de ψ")


# # --- Error de estimación ---
# errorEstimacion = X - X_gorro

# p4 = plot(t_sim, errorEstimacion[3, :],
#     label="Error θ",
#     linewidth=2)

# xlabel!(p4, "Tiempo [s]")
# ylabel!(p4, "Error [rad]")
# title!(p4, "Error de estimación")


# # --- Salida ψ vs referencia ---
# p5 = plot(t_sim, X[5, :],
#     label="Salida ψ",
#     linewidth=2)

# plot!(t_sim, ref,
#     label="Referencia",
#     linestyle=:dash)


# fig = plot(p1, p2, p3, p4,
#            layout=(2,2),
#            size=(1080,720),
#            legend=:topright)

# display(fig)
# #
# # fig = plot(p5,
# #            legend=:topright)
# #
# # display(fig)