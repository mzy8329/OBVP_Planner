from sympy import symbols, diff, integrate, simplify, nsimplify, collect, Poly, Matrix, pprint

t = symbols('t')
T = symbols('T')
q0, qe = symbols('q0 qe')
v0, ve = symbols('v0 ve')
a0, ae = symbols('a0 ae')

alpha = symbols('alpha')
beta = symbols('beta')
gamma = symbols('gamma')
omega = symbols('omega')

qt = alpha/120*t**5 + beta/24*t**4 + gamma/6*t**3 + a0/2*t**2 + v0*t + q0
vt = diff(qt, t)
at = diff(vt, t)
jt = diff(at, t)
temp = jt**2

J = 1/T * integrate(temp, (t, 0, T)) + T * omega
J = simplify(J)
J = collect(J, T)
pprint(J)

end_constrain_dim = 1
# end (x, v, a) constrain
M = Matrix([
    [1/120*T**5, 1/24*T**4, 1/6*T**3],
    [1/24*T**4, 1/6*T**3,  1/2*T**2],
    [1/6*T**3,  1/2*T**2,  1*T]
])
# end (x) constrain
if end_constrain_dim == 1:
    M = Matrix([
        [1/120*T**5, 1/24*T**4, 1/6*T**3],
        [1/6*T**4-2*T, 1/2*T**3-1, T**2],
        [1/2*T**3+T**2,  T**2 + 2*T,  1*T + 2]
    ])
M_inv = M.inv().applyfunc(nsimplify)

dq = qe - a0/2*T**2 - v0*T - q0
dv = ve - a0*T - v0
da = ae - a0
dS = Matrix([dq, dv, da])
Ralpha = simplify((M_inv[0, :] @ dS)[0])
Rbeta = simplify((M_inv[1, :] @ dS)[0])
Rgamma = simplify((M_inv[2, :] @ dS)[0])
# end (x) constrain
if end_constrain_dim == 1:
    dq = qe - a0/2*T**2 - v0*T - q0
    dv = -a0*T
    da = 0
    Ralpha = simplify((M_inv[0, 0] * dq))
    Rbeta = simplify((M_inv[1, 0] * dq))
    Rgamma = simplify((M_inv[2, 0] * dq))


print("alpha: ")
pprint(Ralpha)
print("beta: ")
pprint(Rbeta)
print("gamma: ")
pprint(Rgamma)
print("=====================")

J = J.subs({
    alpha:Ralpha,
    beta: Rbeta,
    gamma: Rgamma
})
J = collect(J, T)

gradJ = diff(J, T)
gradJ = simplify(gradJ)
gradJ = collect(gradJ, T)
print("gradJ: ")
pprint(gradJ)
if end_constrain_dim == 1:
    gradJ = gradJ* 3*T**7*(T**9 + 126*T**6 + 5292*T**3 + 74088)
    dim = 16
else:
    dim = 7
    gradJ = gradJ * T**7

gradJ = simplify(gradJ)
gradJ = collect(gradJ, T)

poly_gradJ = Poly(gradJ, T)
print("poly_gradJ: ")
pprint(poly_gradJ)

poly_coeffs = poly_gradJ.coeffs()

eigen_Matrix = Matrix([[0 for _ in range(dim)] for _ in range(dim)])
eigen_symbol_Matrix = Matrix([[0 for _ in range(dim)] for _ in range(dim)])
print("c")
pprint(poly_coeffs[0])
pprint("--------------")
for i in range(dim):
    if(i < dim-2):
        eigen_Matrix[0, i] = simplify(-poly_coeffs[i+1]/poly_coeffs[0])
        print(i, (eigen_Matrix[0, i]))
    if(i > 1):
        eigen_symbol_Matrix[0, i] = symbols(f"x{i}")

for i in range(dim-1):
    eigen_Matrix[i+1, i] = 1
    eigen_symbol_Matrix[i+1, i] = 1


pprint(eigen_symbol_Matrix)

eigen_value = eigen_symbol_Matrix.eigenvals()
eigen_value = simplify(eigen_value)

for eig_val, mult in eigen_value.items():    
    print(eig_val)
