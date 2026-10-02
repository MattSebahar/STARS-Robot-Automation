import fenics as fe

# -----------------------------------------------------------
# 1. MESH & FUNCTION SPACES
# -----------------------------------------------------------
# BoxMesh defines the physical geometry of the material sample.
mesh = fe.BoxMesh(fe.Point(0, 0, 0), fe.Point(10.0, 1.0, 1.0), 20, 5, 5)

# V: Continuous space for calculating displacement at the mesh nodes.
V = fe.VectorFunctionSpace(mesh, "CG", 1)

# T and W: Discontinuous Galerkin (DG) spaces for history variables.
# Plasticity happens locally inside the material, not across nodes, 
# so we use DG degree 0 (element-wise constants) to store the material's memory.
T = fe.TensorFunctionSpace(mesh, "DG", 0)
W = fe.FunctionSpace(mesh, "DG", 0)

# -----------------------------------------------------------
# 2. MATERIAL PROPERTIES (GRX-810 Estimates)
# -----------------------------------------------------------
# OPTIMIZATION TARGETS: These are the exact parameters the Bayesian 
# Optimizer will tweak to match the UR3e physical test data.

E = 190.6e9     # Young's Modulus (Pa). Controls initial elastic stiffness.
nu = 0.24       # Poisson's ratio. Controls width contraction during stretching.
sig0 = 1000e6   # Initial Yield Stress (Pa). The exact point plastic flow begins.
H = 10e9        # Hardening Modulus (Pa). How much stronger the material gets 
                # as it deforms plastically (strain hardening).

# Convert E and nu to Lamé parameters
mu = E / (2.0 * (1.0 + nu))
lmbda = E * nu / ((1.0 + nu) * (1.0 - 2.0 * nu))

# -----------------------------------------------------------
# 3. STATE & HISTORY VARIABLES
# -----------------------------------------------------------
u = fe.Function(V, name="Total Displacement")
du = fe.TrialFunction(V)
v = fe.TestFunction(V)

# These store the material's permanent deformation across time steps.
eps_p = fe.Function(T, name="Plastic Strain Tensor")
p = fe.Function(W, name="Cumulative Plastic Strain")

d = mesh.geometry().dim()
I = fe.Identity(d)

def eps(u):
    """Calculates the total strain tensor from displacement."""
    return fe.sym(fe.grad(u))

# -----------------------------------------------------------
# 4. RETURN MAPPING ALGORITHM (J2 Plasticity)
# -----------------------------------------------------------
def stress_and_plasticity(u, eps_p, p):
    """
    Calculates the true stress by splitting elastic and plastic strain.
    If the stress exceeds the yield limit, it forces the stress back down 
    to the yield surface (Return Mapping) and updates the plastic memory.
    """
    # 1. Elastic Predictor: Assume no new plastic yielding occurred.
    eps_elastic = eps(u) - eps_p
    sigma_trial = 2.0*mu*eps_elastic + lmbda*fe.tr(eps_elastic)*I
    
    # 2. Deviatoric Stress: The specific part of the stress causing shape change.
    s_trial = sigma_trial - (1.0/3.0)*fe.tr(sigma_trial)*I
    
    # Von Mises equivalent stress (added 1e-14 to prevent division by zero errors)
    sig_eq = fe.sqrt((3.0/2.0)*fe.inner(s_trial, s_trial) + 1e-14)
    
    # 3. Yield Function: Is the stress higher than the hardened yield limit?
    f_yield = sig_eq - (sig0 + H*p)
    
    # 4. Plastic Corrector (The "Return Mapping")
    # If f_yield > 0, the material yielded. Calculate the plastic multiplier (dp).
    dp = fe.conditional(f_yield > 0, f_yield / (3.0*mu + H), 0.0)
    
    # Update the scalar plastic strain (adds to the total)
    p_new = p + dp
    
    # Update the directional plastic strain tensor
    n_dir = s_trial / sig_eq
    eps_p_new = eps_p + (3.0/2.0)*dp*n_dir
    
    # 5. Final Stress Calculation: Calculate stress using the corrected elastic strain.
    eps_elastic_new = eps(u) - eps_p_new
    sigma_new = 2.0*mu*eps_elastic_new + lmbda*fe.tr(eps_elastic_new)*I
    
    return sigma_new, eps_p_new, p_new

# -----------------------------------------------------------
# 5. VARIATIONAL PROBLEM & SOLVER SETUP
# -----------------------------------------------------------
sigma_new, eps_p_new, p_new = stress_and_plasticity(u, eps_p, p)

# The weak form: Internal forces must balance external forces
residual = fe.inner(sigma_new, eps(v)) * fe.dx

# FEniCS automatically derives the tangent stiffness matrix for the Newton solver
Jacobian = fe.derivative(residual, u, du)

# Boundary Conditions: the left grip is stationary and the right grip pulls.
disp_right = fe.Expression(("d_val", "0.0", "0.0"), d_val=0.0, degree=1)

def left_boundary(x, on_boundary): return on_boundary and fe.near(x[0], 0.0)
def right_boundary(x, on_boundary): return on_boundary and fe.near(x[0], 10.0)

bc_left = fe.DirichletBC(V, fe.Constant((0.0, 0.0, 0.0)), left_boundary)
bc_right = fe.DirichletBC(V, disp_right, right_boundary)
bcs = [bc_left, bc_right]

# Define the nonlinear solver
problem = fe.NonlinearVariationalProblem(residual, u, bcs, Jacobian)
solver = fe.NonlinearVariationalSolver(problem)

# -----------------------------------------------------------
# 6. TIME-STEPPING LOOP (The Pull Execution)
# -----------------------------------------------------------
time_steps = 50
max_displacement = 2.0  # Right-grip travel and total extension in mesh units.

# Prepare output file for ParaView visualization
file_results = fe.File("grx810_plasticity/results.pvd")

for t in range(1, time_steps + 1):
    # 1. Command the "robotic arm" to pull slightly further
    current_disp = max_displacement * (t / time_steps)
    disp_right.d_val = current_disp
    print(f"Step {t}/{time_steps}: right-grip travel {current_disp:.3f} units")
    
    # 2. Run the Newton-Raphson solver to find equilibrium
    solver.solve()
    
    # 3. Extract the new plastic history and overwrite the old history variables.
    # This commits the permanent deformation to the material's "memory".
    eps_p.assign(fe.project(eps_p_new, T))
    p.assign(fe.project(p_new, W))
    
    # 4. Save displacement and plastic history for ParaView inspection
    file_results << (u, float(t))
    file_results << (p, float(t))
    file_results << (eps_p, float(t))

print("Simulation complete. Load 'grx810_plasticity/results.pvd' into ParaView.")