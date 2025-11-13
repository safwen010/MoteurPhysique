# Pure Physics Simulation - Technical Documentation

## Overview
This project implements a **constrained rigid body dynamics simulation** from scratch, without using Unity's built-in physics engine. All physics calculations are performed manually using fundamental theorems and equations.

## Implementation Reference
Based on academic papers and industry-standard physics engines:
- **Open Dynamics Engine (ODE)** - Smith 2007
- **Bullet Physics** - Coumans 2014
- Constrained rigid body dynamics formulation (see attached MP.png reference)

---

## Core Physics Components

### 1. Matrix4x4Math.cs - Manual Transformations
**Purpose**: Implement all transformation mathematics without Unity's built-in functions.

#### 4×4 Transformation Matrices
- **Translation Matrix**: T(x,y,z) - moves objects in 3D space
- **Rotation Matrices**: R_x(θ), R_y(θ), R_z(θ) - rotation around each axis
- **Quaternion to Matrix**: Converts orientation to rotation matrix
- **Matrix Composition**: Multiply matrices to combine transformations

#### 3×3 Operations (for Inertia)
- **Rotation matrices** for transforming inertia tensor
- **Matrix multiplication** and **transpose** operations
- **Vector transformation**: Apply rotation to vectors

**Key Equations**:
```
Rotation from quaternion q = (w, x, y, z):
┌                                      ┐
│ 1-2(y²+z²)   2(xy-wz)    2(xz+wy)   │
│ 2(xy+wz)    1-2(x²+z²)   2(yz-wx)   │
│ 2(xz-wy)    2(yz+wx)    1-2(x²+y²)  │
└                                      ┘
```

---

### 2. Fragment.cs - Rigid Body Dynamics
**Purpose**: Each fragment is a complete 6-DOF (six degrees of freedom) rigid body.

#### Physical Properties
- **Mass** (m): Calculated from density × volume or set manually
- **Size**: Dimensions used for inertia calculation
- **Inertia Tensor** (I): Resistance to rotation (3×3 matrix)

#### State Variables
- **Position**: r(t) - location in world space
- **Orientation**: q(t) - quaternion representing rotation
- **Linear Velocity**: v(t) = dr/dt (m/s)
- **Angular Velocity**: ω(t) (rad/s)
- **Linear Momentum**: p = mv
- **Angular Momentum**: L = Iω

#### Force and Torque
- **Force accumulator**: F_total = ΣF (Newtons)
- **Torque accumulator**: τ_total = Στ (N⋅m)

**Key Physics Equations**:

**Inertia Tensor for Uniform Cube**:
```
I_x = (1/12) × m × (h² + d²)
I_y = (1/12) × m × (w² + d²)
I_z = (1/12) × m × (w² + h²)

For a cube of side length a:
I = (1/6) × m × a² × Identity
```

**World-Space Inertia Tensor**:
```
I_world = R × I_local × R^T

where R is the rotation matrix from orientation quaternion
```

**Force at Point**:
```
F applied at point p generates:
- Linear force: F
- Torque: τ = r × F
  where r = p - center_of_mass
```

**Velocity at Point**:
```
v_point = v_com + ω × r
```

**Kinetic Energy**:
```
KE = (1/2)mv² + (1/2)ω·(I·ω)
   = translational + rotational
```

---

### 3. Constraint.cs - Spring-Damper Constraints
**Purpose**: Model connections between fragments as stiff springs that can rupture.

#### Constraint Parameters
- **Spring constant** (k): Stiffness in N/m
- **Damping constant** (c): Damping in N⋅s/m
- **Rest length** (L₀): Natural length of constraint
- **Rupture threshold**: Maximum deformation before breaking

#### Constraint Physics

**Spring-Damper Force**:
```
F = -k(L - L₀) - c(v_rel · n)

where:
- L = current length
- L₀ = rest length
- v_rel = relative velocity
- n = constraint direction (unit vector)
```

**Elastic Potential Energy**:
```
E = (1/2) × k × x²

where x = |L - L₀| (deformation)
```

**Constraint Violation**:
```
φ = L - L₀

If |φ| > threshold → RUPTURE
```

#### Rupture Mechanics

When constraint breaks, stored energy converts to kinetic energy:

**Energy Distribution**:
```
Total energy E splits based on mass ratio:
E_A = E × (m_B / (m_A + m_B))
E_B = E × (m_A / (m_A + m_B))
```

**Impulse Calculation**:
```
From E = (1/2)mv²:
v = √(2E/m)

Impulse J = m × v = √(2Em)
```

**Applied Forces**:
- Linear impulse along constraint direction
- Random angular impulse for realistic tumbling
- Equal and opposite (Newton's 3rd law)

---

### 4. FractureManager.cs - Physics Integration
**Purpose**: Main simulation loop implementing constrained dynamics solver.

#### Integration Method: Semi-Implicit Euler
More stable than explicit Euler for stiff systems.

**Algorithm per timestep**:
```
1. Apply external forces (gravity)
2. Apply constraint forces (springs)
3. Integrate velocities: v_new = v_old + (F/m) × Δt
4. Integrate positions: x_new = x_old + v_new × Δt
5. Handle collisions
```

#### Physics Step Breakdown

**Step 1: External Forces**
```
F_gravity = m × g
τ = 0 (gravity acts at center of mass)
```

**Step 2: Constraint Forces**
Apply spring-damper forces from all active constraints.

**Step 3: Velocity Integration**
```
Linear:
a = F/m
v_new = v_old + a × Δt

Angular:
α = I^(-1) × τ
ω_new = ω_old + α × Δt
```

**Step 4: Position Integration**
```
Linear:
x_new = x_old + v × Δt

Angular (quaternion integration):
q̇ = (1/2) × [0, ω] × q
q_new = q_old + q̇ × Δt
q_new = normalize(q_new)
```

**Step 5: Collision Response**

**Ground Collision**:
```
If penetration detected:
1. Position correction: push out of ground
2. Velocity reflection with restitution:
   v_n_new = -e × v_n_old
   where e = coefficient of restitution
3. Friction on tangential velocity:
   v_t_new = v_t_old × (1 - μ)
   where μ = friction coefficient
```

**Fragment Collision**:
```
Impulse-based resolution:
J = -(1 + e) × v_rel · n / (1/m_A + 1/m_B)

Apply:
v_A += J/m_A
v_B -= J/m_B
```

**Impact Fracture**:
```
If |v_impact| > threshold:
1. Break connected constraints
2. Apply explosive forces
3. Add random angular momentum
```

#### Numerical Stability Features
- Fixed timestep integration
- Damping to prevent energy buildup
- Position correction for penetration
- Constraint force limiting

---

### 5. FracturedCubeGenerator.cs - Geometry Setup
**Purpose**: Generate pre-fractured cube with proper physical properties.

#### Fragment Generation
Creates a grid of small cubes, each as independent rigid body.

**Mass Calculation**:
```
Option 1 (Density-based):
Volume V = a³  (for cube of side a)
Mass m = ρ × V

Option 2 (Fixed mass):
m = constant for all fragments
```

**Grid Layout**:
```
For gridSize = n:
Total fragments = n³

Positions:
for x in [0, n):
  for y in [0, n):
    for z in [0, n):
      position = start + (x, y, z) × (cubeSize + spacing)
```

---

## Physics Parameters Tuning Guide

### Constraint Parameters
- **springConstant** (k): 100-1000 N/m
  - Higher = stiffer, more rigid
  - Lower = softer, more deformation
  
- **dampingConstant** (c): 1-50 N⋅s/m
  - Higher = more stable, less oscillation
  - Lower = springier behavior

- **ruptureThreshold**: 0.1-1.0 m
  - Lower = fragments break easier
  - Higher = stronger connections

### Integration Parameters
- **timeStep**: 0.01-0.02 s
  - Smaller = more accurate, slower
  - Larger = faster, less stable

- **gravity**: -9.81 m/s² (Earth standard)

### Collision Parameters
- **groundRestitution**: 0.0-1.0
  - 0 = no bounce (inelastic)
  - 1 = perfect bounce (elastic)

- **groundFriction**: 0.0-1.0
  - 0 = no friction (ice)
  - 1 = maximum friction (rubber)

---

## Mathematical Background

### Newton-Euler Equations of Motion
```
Linear: F = ma
        dp/dt = F
        where p = mv

Angular: τ = Iα
         dL/dt = τ
         where L = Iω
```

### Constrained Dynamics (Lagrange Multipliers)
The system solves:
```
┌       ┐ ┌   ┐   ┌                    ┐
│ M  -J^T│ │ v │ = │ p + hF_ext        │
│ J   Σ  │ │ λ │   │ -(1/h)Γφ - JM^(-1)p│
└       ┘ └   ┘   └                    ┘

Where:
- M = mass/inertia matrix
- J = constraint Jacobian
- λ = Lagrange multipliers (constraint forces)
- Σ = constraint force mixing (compliance)
- Γ = error reduction parameter (Baumgarte stabilization)
- φ = constraint violation
```

This implementation uses simplified spring-damper model as approximation.

### Energy Conservation
```
Total Energy E_total = KE + PE + Constraint_Energy

KE = Σ[(1/2)m_i v_i² + (1/2)ω_i · (I_i · ω_i)]
PE = Σ[m_i g h_i]
Constraint_Energy = Σ[(1/2)k_j x_j²]

Energy is conserved except:
1. Damping (dissipation)
2. Constraint rupture (converts PE → KE)
3. Collisions (restitution < 1)
```

---

## Debugging and Visualization

### Gizmos
- **Yellow lines**: Active constraints
- **Color gradient**: Constraint stress (green → red)
- **Ground plane**: Gray wireframe

### Debug Logs
- Fragment initialization with mass/size
- Constraint creation count
- Rupture events with energy values
- Impact detection with velocities

### Common Issues

**Fragments exploding**:
- Reduce spring constant
- Increase damping
- Decrease timestep

**Constraints not breaking**:
- Reduce rupture threshold
- Increase drop height
- Check mass values

**Unstable simulation**:
- Decrease timestep
- Increase damping
- Check inertia tensor calculation

---

## References

1. **Smith, R. (2007)** - "Open Dynamics Engine (ODE)" - Constrained rigid body simulation
2. **Coumans, E. (2014)** - "Bullet Physics" - Real-time collision detection and constraint solving
3. **Baraff, D. (1997)** - "An Introduction to Physically Based Modeling: Rigid Body Simulation"
4. **Catto, E. (2005)** - "Iterative Dynamics with Temporal Coherence" - Sequential impulse method
5. **Baumgarte, J. (1972)** - "Stabilization of constraints and integrals of motion"

---

## Future Enhancements

1. **Advanced Constraint Solver**:
   - Projected Gauss-Seidel (PGS)
   - Sequential impulse method
   - Warm starting for convergence

2. **Better Integration**:
   - Runge-Kutta 4th order
   - Verlet integration
   - Symplectic integrators

3. **Collision Detection**:
   - Broad phase (spatial hashing)
   - Narrow phase (GJK/EPA)
   - Continuous collision detection

4. **Material Properties**:
   - Stress-strain relationships
   - Plasticity and permanent deformation
   - Fracture propagation

5. **Performance**:
   - SIMD vectorization
   - Multi-threading
   - GPU compute shaders
