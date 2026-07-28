# ============================================================
#                        MODELO LQR
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
# 3. MATRICES DEL SISTEMA 
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


# ============================================================
# 4. CONTROLABILIDAD
# ============================================================

Wr      = ctrb(sys)
rank_Wr = rank(Wr)

# ============================================================
# 5. DISEÑO DEL LQR 
# ============================================================

Q_lqr = diagm([2000.0, 100.0, 20000.0, 700.0, 10000.0, 8000.0])
R_lqr = [2.0  0.0;
         0.0  2.0]

K= lqr(sys_c, Q_lqr, R_lqr)

println("\n===== Ganancias para C =====")

println(K)

# ============================================================
# 6. SISTEMA EN LAZO CERRADO (CONTINUO)
# ============================================================

Acl_c  = A - B*K
sys_cl = ss(Acl_c, B, C, D)   # sistema continuo en lazo cerrado


# ============================================================
# 7. CONDICIONES INICIALES Y VECTOR DE TIEMPO
# ============================================================

# Estado: x = [pos, vel_lin, ángulo, vel_ang, ángulo_giro, vel_giro]
x0 = [0.0, 0.0, deg2rad(10), 0.0, deg2rad(5), 0.0]   # perturbación inicial

Tfinal = 4.0
dt     = 0.001                # paso fino para que la ODE se resuelva bien
t      = 0:dt:Tfinal

# Entrada externa nula (regulación: el control -K x ya está dentro de sys_cl)
u = zeros(2, length(t))


# ============================================================
# 8. SIMULACIÓN CON lsim (continuo)
# ============================================================

y, t_out, x_out, uout = lsim(sys_cl, u, t; x0 = x0)

# Señal de control real aplicada u(t) = -K x(t)
u_ctrl = -K * x_out


# ============================================================
# 9. GRÁFICAS
# ============================================================

labels_x = [L"x\ (m)" L"\dot{x}\ (m/s)" L"\theta\ (rad)" L"\dot{\theta}\ (rad/s)" L"\psi\ (rad)" L"\dot{\psi}\ (rad/s)"]

p1 = plot(t_out, x_out[1,:], label=labels_x[1], lw=2)
p2 = plot(t_out, x_out[2,:], label=labels_x[2], lw=2)
p3 = plot(t_out, x_out[3,:], label=labels_x[3], lw=2)
p4 = plot(t_out, x_out[4,:], label=labels_x[4], lw=2)
p5 = plot(t_out, x_out[5,:], label=labels_x[5], lw=2)
p6 = plot(t_out, x_out[6,:], label=labels_x[6], lw=2)

plot_states = plot(p1, p2, p3, p4, p5, p6, layout=(3,2), size=(900,700),
                    xlabel="t (s)", legend=:topright,
                    title=["Estados del sistema (lazo cerrado continuo)" "" "" "" "" ""])

display(plot_states)

# Señales de control
# plot_u = plot(t_out, u_ctrl[1,:], label=L"u_1", lw=2, xlabel="t (s)",
#               ylabel="Esfuerzo de control", title="Señales de control (LQR continuo)")
# plot!(plot_u, t_out, u_ctrl[2,:], label=L"u_2", lw=2)

# display(plot_u)



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


