from sympy import symbols, diff, integrate, simplify, nsimplify, collect, Poly, Matrix, pprint

t = symbols('t')
T = symbols('T')
q0, qe = symbols('q0 qe')
v0, ve = symbols('v0 ve')
a0, ae = symbols('a0 ae')

alpha = symbols('alpha')
beta = symbols('beta')
gamma = symbols('gamma')

dq = qe - a0/2*T**2 - v0*T - q0
dv = ve - a0*T - v0
da = ae - a0
M = Matrix([
    [1/60*T**5, 1/24*T**4, 1/6*T**3],
    [1/12*T**4, 1/6*T**3,  1/2*T**2],
    [1/3*T**3,  1/2*T**2,  1*T]
])
M_inv = M.inv().applyfunc(nsimplify)
pprint(M_inv)

dS = Matrix([dq, dv, da])

Ralpha = simplify((M_inv[0, :] @ dS)[0])
Rbeta = simplify((M_inv[1, :] @ dS)[0])
Rgamma = simplify((M_inv[2, :] @ dS)[0])

print("alpha: ")
pprint(Ralpha)
print("beta: ")
pprint(Rbeta)
print("gamma: ")
pprint(Rgamma)

qt = alpha/60*t**5 + beta/24*t**4 + gamma/6*t**3 + a0/2*t**2 + v0*t + q0
vt = diff(qt, t)
at = diff(vt, t)
jt = diff(at, t)
temp = jt**2
# temp = at**2
# temp = vt**2 + at**2 + jt**2

Vmin, Vmax = symbols("Vmin Vmax")
Amin, Amax = symbols("Amin Amax")
l1, l2, l3, l4 = symbols("l1 l2 l3 l4")
temp += l1 * (vt - Vmax) + l2 * (-vt - Vmax) + l3 * (at - Amax) + l4*(-at - Amax)
temp = temp.subs({
    alpha:Ralpha,
    beta: Rbeta,
    gamma: Rgamma  
})

J = 1/T * integrate(temp, (t, 0, T)) + T
J = simplify(J)
J = collect(J, T)
J = J.subs({
    alpha:Ralpha,
    beta: Rbeta,
    gamma: Rgamma
})

gradJ = diff(J, T)
gradJ = simplify(gradJ)
gradJ = collect(gradJ, T)
print("gradJ: ")
pprint(gradJ)
gradJ = gradJ*T**7

gradJ = simplify(gradJ)
gradJ = collect(gradJ, T)

poly_gradJ = Poly(gradJ, T)
poly_coeffs = poly_gradJ.coeffs()

eigen_Matrix = Matrix([[0 for _ in range(7)] for _ in range(7)])
eigen_symbol_Matrix = Matrix([[0 for _ in range(7)] for _ in range(7)])
pprint(poly_coeffs[0])
print("c")
for i in range(7):
    if(i < 5):
        eigen_Matrix[0, i] = simplify(-poly_coeffs[i+1]/poly_coeffs[0])
        print((eigen_Matrix[0, i]))
    if(i > 1):
        eigen_symbol_Matrix[0, i] = symbols(f"x{i}")

for i in range(6):
    eigen_Matrix[i+1, i] = 1
    eigen_symbol_Matrix[i+1, i] = 1


pprint(eigen_symbol_Matrix)

eigen_value = eigen_symbol_Matrix.eigenvals()
eigen_value = simplify(eigen_value)

for eig_val, mult in eigen_value.items():    
    print(eig_val)
