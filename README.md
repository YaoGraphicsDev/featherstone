# Featherstone Articulated Body Physics Engine

A C++ articulated body physics engine for interactive demos and experiments.

## Features

### Featherstone-based forward dynamics
Implements RNEA (Recursive Newton–Euler Algorithm) and CRBA (Composite Rigid Body Algorithm), as described in [Rigid Body Dynamics Algorithms](https://link.springer.com/book/10.1007/978-1-4899-7560-7) by Featherstone, at the core of the solver.

### Closed-loop constraint support
Supports closed-loop systems using simple joints as loop-closure constraints. Loop closure is enforced at the acceleration level, with Baumgarte-stabilized velocity and position terms.

### Joint and DoF coupling
Supports coupling across joints and degrees of freedom by modeling coupling as linear constraints between DoFs. Constraints are resolved by parameterizing joint accelerations using the orthogonal complement basis of the constraint matrix.

### Impulse-based contact resolution
Solves contacts using an impulse-based approach, implementing Projected Gauss–Seidel (PGS) with Sequential Impulses (SI) and impulse warm starting. Impulses transmitted across loop-closure joints are also accounted for during collision handling.

### Springs
Solves spring forces using a semi-implicit Euler integration scheme, improving numerical stability and preventing blow-up for stiff springs.

### glTF-based physics asset pipeline
Streamlines physics asset creation and the simulation pipeline by supporting construction of a physics world from glTF physics assets exported from Blender.


## Supported Joints

| Joint Type | Dynamics Model | Support Loop Closure | Support Spring |
|----|---|---|---|
| Revolute | Motion subspace | Y | Y |
| Prismatic | Motion subspace | Y | Y |
| Cylindrical | Motion subspace | Y | Y (both DoFs) |
| Spherical | Motion subspace | Y | N |
| Gear | Coupling 2 revolutes | N | N |
| Rack and Pinion | Coupling a revolute and a prismatic | N | N |
| Screw | Coupling the 2 DoFs of a cylindrical | N | N |
| Worm | Coupling linear DoF of a cylindrical and a revolute | N | N |

## Demos
[Video link](https://youtu.be/xugRWBTrx6c)

## Next Steps

- Expand joint types (e.g., fixed joints, 6-DoF joints)
- Inverse dynamics and inverse kinematics for more sophisticated control
- Contact block solver for improved convergence and performance
- Coulomb friction formulated as a linear complementarity problem (LCP)
- Joint limits with impulse-based enforcement
- Rotational friction models at contact points