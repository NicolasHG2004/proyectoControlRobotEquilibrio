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
Ci = [0.0  1.0  0.0  0.0]
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

tr = 0.5 # s

wn = (2.23 * zeta^2 + 0.036 * zeta + 1.54) / tr  # rad/s

println("wn = ", wn)
println("zeta = ", zeta)
println("SP = ", SP)
println("tr = ", tr)


n = 3
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

snd = (real * 2.5)

println("Polo No Dominante " , snd)

polos = [s1, s2, snd, snd, snd, snd]

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


# A_mat = [
#      D[1]      0.0        0.0      0.0     0.0;  
#      D[2]      D[1]       N[1]     0.0     0.0;  
#      D[3]      D[2]       N[2]     N[1]     0.0; 
#      D[4]      D[3]       N[3]     N[2]    0.0;  
#      0.0       D[4]       0.0      N[3]     N[1];    
# ]

# b_vec = F

# x = A_mat \ b_vec

# A2 = x[1]
# A1 = x[2]
# B2 = x[3]
# B1 = x[4]
# B0 = x[5]
# A0 = 0.0

# println("A2 = ", A2)
# println("A1 = ", A1)
# println("A0 = ", A0)
# println("B2 = ", B2)
# println("B1 = ", B1)
# println("B0 = ", B0)

# # numC = [B2, B1, B0]
# # denC = [A2, A1, 0]


# numC = (1/A2) *([B2, B1, B0])
# denC = (1/A2) *([A2, A1, 0])

# Controlador = tf(numC, denC)

# polosC =ControlSystems.poles(Controlador)
# println("Numerador del Controlador", numC)
# println("Denominador del Controlador", denC)
# println("Polos del Controlador", polosC)


# C_d = c2d(Controlador, Ts, :tustin)

# println("Controlador discreto: ", C_d)
# polosC_d = ControlSystems.poles(C_d)
# println("Polos del controlador discreto: ", polosC_d)

# L = C_d * G_d
# T = feedback(L)

# T_continuo = feedback(Controlador * G)
# polosLC_continuo = ControlSystems.poles(T_continuo)
# println("\n")
# println("Polos lazo cerrado continuo: ", polosLC_continuo)
# println("\n")
# println("Deberían ser iguales a: ", roots(DT))

# polosLC = ControlSystems.poles(T)
# println("Polos lazo cerrado discreto: ", polosLC)



# # ============================================================
# # RESPUESTA AL ESCALÓN
# # ============================================================

# Tf = 5.0
# t = 0:Ts:Tf

# p1 = plot(step(T, t),
#     title = "Respuesta al escalón",
#     xlabel = "Tiempo [s]",
#     ylabel = "Salida",
#     lw = 2
# )

# display(p1)

# # ============================================================
# # SEÑAL DE CONTROL
# # ============================================================

# r = ones(1, length(t))

# y, _, _ = lsim(T, r, t)

# e = r .- y

# u, _, _ = lsim(C_d, e, t)

# # p2 = plot(
# #     t,
# #     vec(u),
# #     title = "Señal de control",
# #     xlabel = "Tiempo [s]",
# #     ylabel = "u(k)",
# #     lw = 2,
# #     label = "u"
# # )

# # display(p2)

# println("u_max = ", maximum(u))
# println("u_min = ", minimum(u))

