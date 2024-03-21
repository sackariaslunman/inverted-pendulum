import sympy as sp 
from sympy.core import Symbol, Expr
import os

def generate_param_vars(n_poles: int) -> tuple[list[Symbol], list[Symbol], list[Symbol], list[Symbol], list[Symbol], Symbol, Symbol]:
    pole_ms = [sp.symbols(f"m{i+1}") for i in range(n_poles)]
    pole_ls = [sp.symbols(f"l{i+1}") for i in range(n_poles)]
    pole_as = [sp.symbols(f"a{i+1}") for i in range(n_poles)]
    pole_ds = [sp.symbols(f"d{i+1}") for i in range(n_poles)]
    pole_Js = [sp.symbols(f"J{i+1}") for i in range(n_poles)]
    mc, g = sp.symbols("mc g")
    return pole_ms, pole_ls, pole_as, pole_ds, pole_Js, mc, g

def generate_dynamic_vars(n_poles: int) -> list[Symbol]:
    pure_s, pure_d_s, pure_dd_s = sp.symbols("s ds dds")
    pure_thetas = [sp.symbols(f"theta{i+1}") for i in range(n_poles)]        #type: ignore
    pure_d_thetas = [sp.symbols(f"dtheta{i+1}") for i in range(n_poles)]    #type: ignore
    pure_dd_thetas = [sp.symbols(f"ddtheta{i+1}") for i in range(n_poles)]  #type: ignore
    pure_vars = [pure_s, pure_d_s] + [item for pair in zip(pure_thetas, pure_d_thetas) for item in pair] + [pure_dd_s] + pure_dd_thetas
    return pure_vars

def calculate_equations_of_motions(n_poles: int, verbose: bool = True):
    assert n_poles > 0

    if verbose:
        print(f"Calculating equations of motion for {n_poles} pole(s)")
        print(f"Creating variables...")

    # Create time dependent variables
    t = sp.symbols("t")
    s = sp.Function("s")(t)
    d_s = sp.diff(s,t)
    dd_s = sp.diff(d_s,t)
    thetas = [sp.Function(f"theta{i+1}")(t) for i in range(n_poles)]
    d_thetas = [sp.diff(theta,t) for theta in thetas]
    dd_thetas = [sp.diff(d_theta,t) for d_theta in d_thetas]
    tau = sp.symbols("tau")

    # Parameter variables
    pole_ms, pole_ls, pole_as, pole_ds, pole_Js, mc, g = generate_param_vars(n_poles)

    pole_pc1s = []
    pole_pc2s = []
    for i, (theta, a) in enumerate(zip(thetas, pole_as)):
        prev_1 = 0
        prev_2 = 0
        for prev_l, prev_theta in list(zip(pole_ls, thetas))[:i]:
            prev_1 += -prev_l*sp.sin(-prev_theta) #type: ignore
            prev_2 += prev_l*sp.cos(-prev_theta) #type: ignore
        pole_pc1s.append(s-a*sp.sin(-theta)+prev_1) #type: ignore
        pole_pc2s.append(a*sp.cos(-theta)+prev_2) #type: ignore

    if verbose:
        print(f"Generating Euler-Lagrange equations...")

    # Kinetic energy
    T = 0.5*mc*d_s**2 #type: ignore
    for m, pc1, pc2, J, d_theta in zip(pole_ms, pole_pc1s, pole_pc2s, pole_Js, d_thetas):
        d_pc1 = sp.diff(pc1,t)
        d_pc2 = sp.diff(pc2,t)
        T += 0.5*m*(d_pc1**2 + d_pc2**2) + 0.5*J*d_theta**2 #type: ignore

    # Potential energy
    V = 0   
    for m, pc2 in zip(pole_ms, pole_pc2s):
        V += g*m*pc2

    # Dissipation function
    R = 0
    prev_w = 0
    for d, d_theta in zip(pole_ds, d_thetas):
        R += 0.5*d*(d_theta-prev_w)**2 #type: ignore
        prev_w = d_theta

    eqs = []
    L = T-V
    lh = sp.diff(sp.diff(L, d_s),t) - sp.diff(L, s) + sp.diff(R, d_s) #type: ignore
    rh = tau
    eqs = [lh-rh]
    for theta, d_theta in zip(thetas, d_thetas):
        lh = sp.diff(sp.diff(L, d_theta),t) - sp.diff(L, theta) + sp.diff(R, d_theta) #type: ignore
        rh = 0
        eqs.append(lh-rh)

    if verbose:
        print(f"Simplifying equations...")
    # Simplify equations makes the solution faster (I think)
    eqs = [sp.simplify(eq) for eq in eqs]

    if verbose:
        print(f"Solving for solutions...")

    # Using the manual flag and not simplifying the solutions makes the solution faster (I think)
    # At the moment, n_poles > 2 never finishes
    manual = True
    sols = sp.solve(eqs, [tau]+dd_thetas,manual=manual,simplify=False)

    if verbose:
        print(f"Simplifying solutions...")
    if manual:
        sols = dict(zip([tau]+dd_thetas, sols[0]))

    sp_sols = [sp.simplify(sols[tau])] + [sp.simplify(sols[dd_theta]) for dd_theta in dd_thetas] 
    sp_vars = [s, d_s] + [item for pair in zip(thetas, d_thetas) for item in pair] + [dd_s] + dd_thetas

    pure_vars = generate_dynamic_vars(n_poles)
    subs_dict = dict(zip(sp_vars, pure_vars))
    pure_sols = [sol.subs(subs_dict) for sol in sp_sols]

    return sp_sols, pure_sols

def save_equations_of_motions(n_poles: int, sols) -> None:
    for i, sol in enumerate(sols):
        name = "tau" if i == 0 else f"dd_theta_{i}"
        current_path = os.path.dirname(__file__)
        file_path = f"{current_path}/solutions/{name}_with_{n_poles}_poles.txt"
        with open(file_path, 'w') as file:
            file.write(str(sol))

def load_equations_of_motions(n_poles: int) -> list[Expr]:
    sols = []
    for i in range(1+n_poles):
        name = "tau" if i == 0 else f"dd_theta_{i}"
        current_path = os.path.dirname(__file__)
        file_path = f"{current_path}/solutions/{name}_with_{n_poles}_poles.txt"
        with open(file_path, 'r') as file:
            expr_str = file.read()
            expr = sp.sympify(expr_str)
            sols.append(expr)
    return sols

def substitute_params(n_poles: int, sols: list, pole_ms: list, pole_ls: list, pole_as: list, pole_ds: list, pole_Js: list, mc: float, g: float) -> list[Expr]:
    param_values = pole_ms + pole_ls + pole_as + pole_ds + pole_Js + [g, mc]
    var_ms, var_ls, var_as, var_ds, var_Js, var_mc, var_g = generate_param_vars(n_poles)
    param_vars = var_ms + var_ls + var_as + var_ds + var_Js + [var_g, var_mc]
    subs_dict = dict(zip(param_vars, param_values))
    sols_with_params = [sol.subs(subs_dict) for sol in sols]
    return sols_with_params

def main() -> None:
    n_poles = 2
    sols, pure_sols = calculate_equations_of_motions(n_poles)
    print("Solutions:")
    for i, sol in enumerate(sols):
        name = "tau" if i == 0 else f"dd_theta_{i}"
        print(f"{name}:")
        print(sol)

if __name__ == "__main__":
    main()