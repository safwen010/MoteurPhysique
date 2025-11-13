# Quick Setup Guide

## Getting Started

### 1. Scene Setup
1. Create an empty GameObject in your scene
2. Name it "FracturedCube"
3. Add the `FracturedCubeGenerator` component
4. Add the `FractureManager` component

### 2. Configure Generator
In the Inspector for `FracturedCubeGenerator`:
- **Grid Size**: 3 (creates 3×3×3 = 27 fragments)
- **Cube Size**: 0.5 (size of each fragment)
- **Spacing**: 0.0 (no gaps between fragments)
- **Density**: 1000 kg/m³ (water-like density)
- **Use Density**: ✓ (auto-calculate mass)
- **Randomize Colors**: ✓ (easier to see individual fragments)

### 3. Configure Physics Manager
In the Inspector for `FractureManager`:

#### Constraint Parameters
- **Spring Constant**: 500 N/m
- **Damping Constant**: 10 N⋅s/m
- **Rupture Threshold**: 0.5 m

#### Integration Parameters
- **Time Step**: 0.02 s
- **Constraint Iterations**: 10
- **Error Reduction Parameter**: 0.2

#### External Forces
- **Gravity**: (0, -9.81, 0) m/s²
- **Air Damping**: 0.995

#### Ground Collision
- **Ground Y**: 0
- **Ground Restitution**: 0.3
- **Ground Friction**: 0.5

#### Initial Conditions
- **Initial Drop Height**: 5 m
- **Initial Velocity**: (0, 0, 0)

### 4. Run the Simulation
Press Play! The cube should:
1. Drop from 5m height
2. Hit the ground
3. Fracture on impact
4. Fragments scatter with realistic physics

## Troubleshooting

### Fragments fly apart immediately
- **Solution**: Increase spring constant to 1000+
- Or decrease rupture threshold to 0.3

### No fracturing on impact
- **Solution**: Decrease rupture threshold to 0.2
- Or increase drop height to 10

### Jittery/unstable motion
- **Solution**: Decrease timestep to 0.01
- Or increase damping to 20

### Too bouncy
- **Solution**: Decrease ground restitution to 0.1
- Or increase ground friction to 0.8

## Advanced Configuration

### Heavier/Lighter Object
Adjust in `FracturedCubeGenerator`:
- **Density**: 500 (light) to 2000 (heavy)

### More/Fewer Fragments
Adjust in `FracturedCubeGenerator`:
- **Grid Size**: 2 (8 fragments) to 5 (125 fragments)
- Note: More fragments = slower simulation

### Stronger/Weaker Constraints
Adjust in `FractureManager`:
- **Spring Constant**: 100 (weak) to 2000 (strong)
- **Rupture Threshold**: 0.1 (brittle) to 2.0 (tough)

## Performance Tips

For better performance:
1. Keep grid size ≤ 4 (64 fragments max)
2. Use timestep = 0.02 (not smaller)
3. Constraint iterations = 5-10 (not more)

## Visualization

Enable Gizmos in Scene view to see:
- **Yellow lines**: Active constraints
- **Color gradient**: Stress on constraints (green = low, red = high)
- **Gray plane**: Ground collision surface

## Next Steps

Try these experiments:
1. Change drop height and observe impact energy
2. Adjust spring constant to see material stiffness
3. Modify gravity (e.g., Moon = -1.62 m/s²)
4. Add initial velocity for projectile motion
5. Change grid size for different fracture patterns
