# ============================================================
#                   Modelo LQR DISCRETO
# ============================================================

using LinearAlgebra
using Printf
using ControlSystems
using Plots
using LaTeXStrings

# ------------------------------------------------------------
# 1. Parámetros físicos del sistema
# ------------------------------------------------------------
m        = 0.035
r        = 0.0672 / 2
inercia  = 0.5 * m * r^2

M        = 1.000 - 2*m
L        = 0.5 * 0.0766

J_centroide = (1/12) * M * (0.0766^2 + 0.0575^2)
d        = 0.1612
J_Y_delta = (1/12) * M * (0.0766^2 + 0.0575^2)
g        = 9.8

# ------------------------------------------------------------
# 2. Términos auxiliares para las matrices A y B
# ------------------------------------------------------------
Q_aux = J_centroide * M + (J_centroide + M*L^2) * (2*m + 2*inercia/r^2)

A_23 = -(M^2 * L^2 * g) / Q_aux
A_43 =  M * L * g * (M + 2*m + 2*inercia/r^2) / Q_aux

B_21 = (J_centroide + M*L^2 + M*L*r) / (Q_aux * r)
B_22 = B_21

B_41 = -(M*L/r + M + 2*m + 2*inercia/r^2) / Q_aux
B_42 = B_41

B_61 =  1 / (r * (m*d + inercia*d/r^2 + 2*J_Y_delta/d))
B_62 = -B_61

# ------------------------------------------------------------
# 3. Matrices del sistema (continuo) y discretización
#    Estado: x = [posición, vel. lineal, ángulo, vel. angular, ángulo giro, vel. giro]
# ------------------------------------------------------------
A = [0  1    0    0  0  0;
     0  0  A_23   0  0  0;
     0  0    0    1  0  0;
     0  0  A_43   0  0  0;
     0  0    0    0  0  1;
     0  0    0    0  0  0]

B = (inercia/r) .* [0    0  ;
                     B_21 B_22;
                     0    0  ;
                     B_41 B_42;
                     0    0  ;
                     B_61 B_62]

C = Matrix{Float64}(I, 6, 6)
D = zeros(6, 2)

Ts = 0.01  

sys_c = ss(A, B, C, D)
sys_d = c2d(sys_c, Ts, :zoh)

Ad, Bd, Cd, Dd = sys_d.A, sys_d.B, sys_d.C, sys_d.D
sys = ss(Ad, Bd, Cd, Dd, Ts)

# ------------------------------------------------------------
# 4. Controlabilidad
# ------------------------------------------------------------
Wr = ctrb(sys)
rank_Wr = rank(Wr)
println("Rango de controlabilidad: ", rank_Wr, " / ", size(Ad,1))

# ------------------------------------------------------------
# 5. Diseño del LQR discreto
# ------------------------------------------------------------
Q_lqr = diagm([7700.0, 0.0, 0.0, 1600.0, 500.0, 0.0])
R_lqr = [1.0  0.0;
         0.0  1.0]

K_d = lqr(sys, Q_lqr, R_lqr)

println("\nMatriz de ganancias K discreta:")
display(K_d)

Acl_d = Ad - Bd * K_d   # Matriz de lazo cerrado DISCRETA

polos_cl = eigvals(Acl_d)
println("\nPolos del lazo cerrado discreto (deben estar dentro del círculo unitario):")
display(polos_cl)
println("¿Estable? ", all(abs.(polos_cl) .< 1))

# ------------------------------------------------------------
# 6. Precompensador kr DISCRETO (seguimiento de x5 = ψ)
# ------------------------------------------------------------
C5 = [0.0  0.0  0.0  0.0  1.0  0.0]   # Salida: x5 (ángulo de giro ψ)

DC_dif_d = (C5 * inv(I - Acl_d) * (Bd[:, 1] - Bd[:, 2]))[1] #Esta formula se usa en discreto
kr_d     = 1.0 / DC_dif_d

@printf "\nPrecompensador kr discreto (seguimiento x5) = %.6f\n" kr_d

# ------------------------------------------------------------
# 7. Sistema de lazo cerrado DISCRETO con seguimiento de x5
# ------------------------------------------------------------
Bd_r  = Bd * [kr_d; -kr_d]             # Vector de entrada efectivo (6x1)
C_all = Matrix{Float64}(I, 6, 6)
D_r   = zeros(6, 1)

sys_lc_d = ss(Acl_d, Bd_r, C_all, D_r, Ts)   # sistema discreto para lsim

# ------------------------------------------------------------
# 8. Simulación con lsim (discreta) — escalón unitario en x5
# ------------------------------------------------------------
t_sim = 0.0:Ts:5.0
ref   = ones(1, length(t_sim))

Y, t_out, X = lsim(sys_lc_d, ref, t_sim, [0.1, 0, 0, 0, 0, 0])

# Reconstruir señales de control: u = -K_d*x + [+kr_d; -kr_d]*r
U = -K_d * X .+ [kr_d; -kr_d] * ref

# ------------------------------------------------------------
# 9. Graficación — 2x2: estados + controles
# ------------------------------------------------------------
p1 = plot(t_out, Y[5, :],
          label = L"x_5\ —\ \psi\ \mathrm{[rad]}",
          color = :purple, linewidth = 2,
          ylabel = "rad / rad·s⁻¹")
plot!(p1, t_out, [1.0 for _ in t_out],
          label = "referencia",
          color = :black, linewidth = 1, linestyle = :dot)
plot!(p1, t_out, Y[6, :],
          label = L"x_6\ —\ \dot{\psi}\ \mathrm{[rad/s]}",
          color = :magenta, linewidth = 2, linestyle = :dash)
title!(p1, "Giro lateral ψ (salida seguida) — discreto")

p2 = plot(t_out, Y[3, :] .* (180/π),
          label = L"x_3\ —\ \theta\ \mathrm{[°]}",
          color = :red, linewidth = 2,
          ylabel = "° / rad·s⁻¹")
plot!(p2, t_out, Y[4, :],
          label = L"x_4\ —\ \dot{\theta}\ \mathrm{[rad/s]}",
          color = :orange, linewidth = 2, linestyle = :dash)
title!(p2, "Ángulo de inclinación θ")

p3 = plot(t_out, Y[1, :],
          label = L"x_1\ —\ \mathrm{posicion\ [m]}",
          color = :blue, linewidth = 2,
          ylabel = "m / m·s⁻¹")
plot!(p3, t_out, Y[2, :],
          label = L"x_2\ —\ v\ \mathrm{[m/s]}",
          color = :teal, linewidth = 2, linestyle = :dash)
title!(p3, "Posición y velocidad lineal")

p4 = plot(t_out, U[1, :],
          label = L"u_1\ —\ \mathrm{motor\ izq.}",
          color = :blue, linewidth = 2,
          ylabel = "N·m (o PWM equiv.)")
plot!(p4, t_out, U[2, :],
          label = L"u_2\ —\ \mathrm{motor\ der.}",
          color = :red, linewidth = 2, linestyle = :dash)
title!(p4, "Señales de control")

fig = plot(p1, p2, p3, p4,
           layout  = (2, 2),
           size    = (1000, 600),
           xlabel  = "Tiempo [s]",
           legend  = :topright,
           plot_title = "Respuesta al escalón en x5 (ψ = 1 rad) — LQR DISCRETO con seguimiento",
           plot_title_fontsize = 11,
           margin  = 5Plots.mm)

display(fig)
savefig(fig, "respuesta_escalon_x5_discreto.png")
println("\nGráfica guardada en: respuesta_escalon_x5_discreto.png")

# ------------------------------------------------------------
# 10. Resumen numérico
# ------------------------------------------------------------
println("\n--- Resumen de la simulación (discreta) ---")
@printf "  Referencia x5       :  1.000000 rad\n"
@printf "  x5 final (t=%.1f s) : %+.6f rad\n"  t_out[end]  Y[5,end]
@printf "  Error en régimen    : %+.6f rad\n"   1.0 - Y[5,end]
@printf "  Sobreimpulso x5     : %+.4f rad (%.2f %%)\n" maximum(Y[5,:])  (maximum(Y[5,:])-1)*100
@printf "  x3 max (inclinación): %+.4f °\n"    maximum(abs.(Y[3,:])) * (180/π)
@printf "  |u| máximo          : %+.4f N·m\n"  maximum(abs.(U))