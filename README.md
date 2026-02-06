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
| Revolute | Motion subspace | ✅ | ✅ |
| Prismatic | Motion subspace | ✅ | ✅ |
| Cylindrical | Motion subspace | ✅ | ✅ *both DoFs |
| Spherical | Motion subspace | ✅ | ❌ |
| Gear | Coupling 2 revolutes | ❌ | ✅ *both  joints |
| Rack and Pinion | Coupling a revolute and a prismatic | ❌ | ✅ *both joints |
| Screw | Coupling the 2 DoFs of a cylindrical | ❌ | ✅ *both DoFs |
| Worm | Coupling linear DoF of a cylindrical and a revolute | ❌ | ✅ *all DoFs and joints |

- A spring cannot be applied to a joint when the joint is a loop closure.

## Demos

[YouTube link](https://youtu.be/xugRWBTrx6c)

What each of these demos demonstrates
| Name | Demo | Articulated | Loop Closure | Spring | Coupling | Spherical Joint | Trivial PID Control |
|---|---|---|---|---|---|---|---|
| Scissor Lift | <img src="media/scissor_lift.gif" width="640"/> | ✅ | ✅ | ❌ | ❌ | ❌ | ❌ |
| Worm Screw Jack | <img src="media/worm_screw_jack.gif" width="640"/>  | ✅ | ✅ | ❌ | ✅*heavy | ❌ | ✅ |
| Spring Scale | <img src="media/spring_scale.gif" width="640"/> | ✅ | ❌ | ✅*stiff | ✅ | ❌ | ❌ |
| Double Wishbone Suspension | <img src="media/double_wishbone.gif" width="640"/> | ✅ | ✅ |  ❌ |✅ | ✅ | ✅ |
| Collision and Scene Test | <img src="media/wrecking_ball.gif" width="640"/> | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ |
| Spring Test | <img src="media/spring_test.gif" width="640"/> | ✅ | ✅ | ✅*stiff | ❌ | ❌ | ❌ |
| Spherical Joint Test | <img src="media/spherical_test.gif" width="640"/> | ✅ | ✅ | ❌ | ❌ | ✅ | ❌ |

## Next Steps
Articulated Body Physics Engine Demos

YouTube doesn't allow me to post active links due to my channel being relatively new.
Any link pasted here also gets cut short.
So please go search my profile and repo on GitHub, thanks.

GitHub profile: YaoGraphicsDev
Repository: featherstone

- Expand joint types (e.g., fixed joints, 6-DoF joints)
- Inverse dynamics and inverse kinematics for more sophisticated control
- Contact block solver for improved convergence and performance
- Coulomb friction formulated as a linear complementarity problem (LCP)
- Joint limits with impulse-based enforcement
- Rotational friction models at contact points