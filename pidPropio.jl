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
#tfangulo = tf(sys_c)

Ts = 0.01

# Discretización del sistema
sys_d = c2d(sys_c, Ts, :tustin)

Ad, Bd, Cd, Dd = sys_d.A, sys_d.B, sys_d.C, sys_d.D

sys = ss(Ad, Bd, Cd, Dd, Ts)

G = tf(sys)
println("La función de transferencia discreta SISO es ", G)

SP = 5

zeta = abs(log(SP / 100)) / sqrt(log(SP / 100)^2 + π^2)

tr = 0.2 # s

wn = (2.23 * zeta^2 + 0.036 * zeta + 1.54) / tr  # rad/s

println("wn = ", wn)
println("zeta = ", zeta)
println("SP = ", SP)
println("tr = ", tr)


n = 2
vl = 1
m = 2
q = 2*n - 1 + vl


println("Polos del lazo cerrado ", q)
real = -zeta*wn
imag = wn * sqrt.(1 -  zeta^2)
s1 = real +1im*imag
s2 = real - 1im*imag

println("Polo 1 es: ", s1)
println("Polo 2 es: ", s2)

snd = (real * 2)

println("Polo No Dominante " , snd)

z1  = exp(s1  * Ts)
z2  = exp(s2  * Ts)
znd = exp(snd * Ts)

polos = [z1, z2, znd, znd]
DT = fromroots(polos)

println("Polos de DT:")
println(roots(DT))
F = reverse(coeffs(DT))
println(DT)

println(size(DT))
polosG =ControlSystems.poles(G)

println("Polos FT: ", polosG)

num = G.matrix[1].num
den = G.matrix[1].den

D = reverse(coeffs(den))
N = reverse(coeffs(num))
println("Orden de Numerador", size(N))

println("Numerador: ",N)
println("Denominador: ", D)

n1 = N[1]   # coeficiente de z
n0 = N[2]   # término independiente

A_mat = [
     D[1]      0.0        0.0     0.0      0.0     0.0;   # z^4
     D[2]      D[1]       0.0     n1       0.0     0.0;   # z^3
     D[3]      D[2]       D[1]    n0       n1      0.0;   # z^2
     0.0       D[3]       D[2]    0.0      n0      n1;    # z^1
     0.0       0.0        D[3]    0.0      0.0     n0;    # z^0
     1.0       1.0        1.0     0.0      0.0     0.0    # A2+A1+A0 = 0 (integrador)
]

b_vec = [F[1], F[2], F[3], F[4], F[5], 0.0]   # el 0.0 es el lado derecho de la 6ta ecuación

x = A_mat \ b_vec

A2 = x[1]
A1 = x[2]
A0 = x[3]
B2 = x[4]
B1 = x[5]
B0 = x[6]

println("A2 = ", A2)
println("A1 = ", A1)
println("A0 = ", A0)
println("B2 = ", B2)
println("B1 = ", B1)
println("B0 = ", B0)

numC = [B2, B1, B0]
denC = [A2, A1, A0]     # antes era [A2, A1, 0]

Controlador = tf(numC, denC, Ts)

polosC =ControlSystems.poles(Controlador)
println("Numerador del Controlador", numC)
println("Denominador del Controlador", denC)
println("Polos del Controlador", polosC)


L = Controlador*G

T = feedback(L)

polosLC = ControlSystems.poles(T)
println("Polos lazo cerrado:")
println(polosLC)



# # ============================================================
# # 4. CONTROLABILIDAD
# # ============================================================

# Wr      = ctrb(sys)
# rank_Wr = rank(Wr)
# println("Rango de controlabilidad: ", rank_Wr, " / ", size(Ad, 1))


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

# ============================================================
# 4. SIMULACIÓN POR ECUACIONES EN DIFERENCIAS
# ============================================================

tfinal = 3.0
N_steps = round(Int, tfinal / Ts) + 1
t_vec = (0:N_steps-1) .* Ts

r = ones(N_steps)
y = zeros(N_steps)
u = zeros(N_steps)
e = zeros(N_steps)

D1, D2, D3 = D[1], D[2], D[3]

for k in 3:N_steps
    y[k] = -D2*y[k-1] - D3*y[k-2] + n1*u[k-1] + n0*u[k-2]
    e[k] = r[k] - y[k]
    u[k] = (B2*e[k] + B1*e[k-1] + B0*e[k-2] - A1*u[k-1] - A0*u[k-2]) / A2
end

p1 = plot(t_vec, y, label="Salida y(t)", lw=2,
          xlabel="Tiempo [s]", ylabel="Ángulo [rad]",
          title="Respuesta al escalón - Lazo cerrado")
hline!([1.0], label="Referencia", ls=:dash, color=:black)

p2 = plot(t_vec, u, label="Esfuerzo de control u(t)", lw=2,
          xlabel="Tiempo [s]", ylabel="u", color=:purple,
          title="Señal de control")

plot(p1, p2, layout=(2,1), size=(700,600))