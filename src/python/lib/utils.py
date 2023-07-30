def sympy2casadi(sympy_expr,sympy_var,casadi_var):
  import casadi
  assert casadi_var.is_vector()
  if casadi_var.shape[1]>1:
    casadi_var = casadi_var.T
  casadi_var = casadi.vertsplit(casadi_var)
  from sympy.utilities.lambdify import lambdify

  mapping = {'ImmutableDenseMatrix': casadi.blockcat,
             'MutableDenseMatrix': casadi.blockcat,
             'Abs':casadi.fabs
            }
  f = lambdify(sympy_var,sympy_expr,modules=[mapping, casadi])
  return f(*casadi_var)