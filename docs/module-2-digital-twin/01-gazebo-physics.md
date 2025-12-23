---
sidebar_position: 2
title: "Chapter 1: Gazebo Physics Simulation"
description: "Configure Gazebo physics engines (ODE, Bullet) for realistic humanoid robot simulation with gravity, collisions, and ROS 2 integration"
---

# Chapter 1: Gazebo Physics Simulation

**Module**: The Digital Twin (Gazebo & Unity)
**Estimated Reading Time**: 25 minutes
**Prerequisites**: Module 1 (ROS 2 fundamentals, URDF), basic physics (Newton's laws, friction)

---

## Learning Objectives

By the end of this chapter, you will be able to:
- Compare Gazebo's physics engines (ODE, Bullet, DART) and select the appropriate engine for humanoid simulation
- Configure physics parameters (`max_step_size`, `real_time_factor`, gravity) to balance accuracy and performance
- Set contact properties (friction, restitution, damping) to match real-world materials
- Integrate Gazebo with ROS 2 for model spawning and joint control

---

## Prerequisites

**Knowledge Prerequisites**:
- Understand ROS 2 topics and launch files (Module 1, Chapter 1)
- Familiarity with URDF robot models (Module 1, Chapter 2)
- Basic physics: Newton's laws (F=ma), gravitational acceleration, friction forces

**Software Prerequisites**:
- Gazebo Classic 11 installed (`gazebo --version`)
- ROS 2 Humble with `gazebo_ros_pkgs` (`ros2 pkg list | grep gazebo`)
- Verification: `ros2 launch gazebo_ros gazebo.launch.py` opens empty Gazebo world

---

## Introduction

Testing humanoid robots in the real world risks expensive hardware damage from falls or collisions. Physics simulation provides a safe virtual environment where robots can fail without consequence. Gazebo Classic 11 uses numerical integration to model how forces (gravity, collisions, torques) affect robot motion, enabling validation of control algorithms before hardware deployment (Koenig & Howard, 2004).

In this chapter, you'll configure Gazebo's physics engines, tune parameters (gravity, timesteps, friction), and integrate with ROS 2 for model spawning and control.

---

## Section 1: Gazebo Architecture and Physics Engines

### Physics Engine Overview

Gazebo supports three engines:

1. **ODE**: Fast, mature, best for prototyping and real-time applications
2. **Bullet**: Accurate impulse-based solver, better joint stability, use for final validation
3. **DART**: Deterministic with analytical derivatives, best for optimization research

**Recommendation**: Use **ODE** for humanoid development (fast iteration), validate with **Bullet** (accuracy) before hardware deployment (Koenig & Howard, 2004).

### Selecting a Physics Engine

In Gazebo world files (`.world` or `.sdf`), specify the engine in the `<physics>` tag:

```xml
<physics name="default_physics" type="ode"> <!-- or "bullet", "dart" -->
  <!-- Physics parameters here -->
</physics>
```

**Key Point**: Changing physics engines may require retuning contact properties (friction, damping) because solvers use different numerical methods.

---

## Section 2: Physics Parameters

### Critical Parameters

#### Key Parameters

1. **Gravity**: `0 0 -9.81` m/s² (Earth). Lower gravity (e.g., Moon: `0 0 -1.62`) reduces balance difficulty but changes gait dynamics.

2. **Max Step Size**: Physics timestep in seconds. Use 0.001s (default) for balance accuracy/speed. Smaller (0.0001s) for fast collisions, larger (0.01s) risks instability.

3. **Real-Time Factor**: `1.0` = real-time, `0.5` = half-speed. Monitor in Gazebo GUI - if actual < target, simplify meshes or increase timestep.

4. **Solver Iterations**: Constraint iterations/timestep. Default 50 for most robots, 200+ for precision tasks (grasping).

**Example Configuration**:
```xml
<physics name="default_physics" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <gravity>0 0 -9.81</gravity>
  <ode>
    <solver>
      <type>quick</type>       <!-- Fast LCP solver -->
      <iters>50</iters>         <!-- Constraint iterations -->
      <sor>1.3</sor>            <!-- Successive over-relaxation parameter -->
    </solver>
  </ode>
</physics>
```

**Validation**: Drop a 1 kg box from 1 meter. At Earth gravity, it should hit the ground at `t = sqrt(2h/g) = sqrt(2*1/9.81) ≈ 0.45s` and achieve velocity `v = sqrt(2gh) ≈ 4.43 m/s`. Physics engine accuracy can be benchmarked against analytical predictions (Pitonakova et al., 2018).

---

## Section 3: Contact and Collision Properties

### Material Properties

Contact behavior (bouncing, sliding, sticking) is controlled by surface properties in URDF `<collision>` tags.

#### Contact Properties

1. **Friction** (`<mu>`, `<mu2>`): 0.0 (ice) to 2.0+ (rubber on concrete). Use 0.8-1.0 for humanoid feet on floors.

2. **Restitution**: Bounciness (0.0 = inelastic, 1.0 = elastic). Set 0.0 for robot links (no bouncing).

3. **Contact Stiffness/Damping**: `kp=1e6` (rigid), `kd=1.0` (light damping) for robot bodies.

**Example URDF**:
```xml
<collision name="foot_collision">
  <surface>
    <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
    <bounce><restitution_coefficient>0.0</restitution_coefficient></bounce>
    <contact><ode><kp>1e6</kp><kd>1.0</kd></ode></contact>
  </surface>
</collision>
```

### Common Issues and Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| Robot foot penetrates ground | Low contact stiffness (`kp`) | Increase `<kp>` to 1e6 or higher |
| Joints vibrate/oscillate | High contact stiffness, low damping | Increase `<kd>` to 1.0-10.0 |
| Robot slides unexpectedly | Low friction coefficients | Increase `<mu>` and `<mu2>` to 0.8-1.0 |
| Simulation runs slowly | Timestep too small, too many contacts | Increase `<max_step_size>` to 0.002s or simplify collision meshes |
| Robot explodes/flies apart | Timestep too large, solver unstable | Reduce `<max_step_size>` to 0.001s, increase `<iters>` to 100 |

---

## Section 4: Integrating with ROS 2

### ROS 2 Gazebo Packages

The `gazebo_ros_pkgs` suite provides ROS 2 integration (Staranowicz & Mariottini, 2011):

1. **`gazebo_ros`**: Launch Gazebo server and GUI
2. **`gazebo_ros_pkgs`**: Spawn models, delete entities, get/set physics properties
3. **`gazebo_ros2_control`**: Hardware interface for `ros2_control` controllers

**Installation** (if not already installed):
```bash
sudo apt install ros-humble-gazebo-ros-pkgs -y
```

### Launching and Spawning

**Launch Gazebo**:
```bash
ros2 launch gazebo_ros gazebo.launch.py world:=path/to/custom.world
```

**Spawn Robot**:
```bash
ros2 run gazebo_ros spawn_entity.py -entity humanoid -file robot.urdf -z 1.0
```

**Key Topics**:
- `/clock`: Simulation time
- `/joint_states`: Joint positions/velocities
- `/gazebo/link_states`: All link poses

### Testing and Verification

**Checkpoint Verification**:
```bash
# Launch Gazebo with custom physics
ros2 launch my_package launch_gazebo.launch.py

# Check simulation is running
ros2 topic echo /clock

# Spawn robot
ros2 run gazebo_ros spawn_entity.py -entity test_box -file box.urdf

# Verify robot appears in Gazebo GUI
# Expected: Box falls due to gravity, collides with ground plane
```

**Success Criteria**:
- Gazebo launches without errors
- Robot spawns at specified pose (x, y, z)
- Robot falls under gravity at 9.81 m/s²
- Ground collision prevents penetration
- `/clock` topic publishes simulation time

---

## Practice Exercises

### Exercise 1.1: Change Gravity to Moon (Beginner)
**Objective**: Modify Gazebo world file to simulate lunar gravity (1.62 m/s²) and observe robot falling slower.

**Hint**: Edit `<gravity>` tag in world file, relaunch Gazebo, time how long a 1 kg box takes to fall 1 meter. Compare to Earth gravity (0.45s vs ~1.08s expected).

**Estimated Time**: 15 minutes

---

### Exercise 1.2: Tune Friction for Sliding vs Gripping (Intermediate)
**Objective**: Configure foot collision friction coefficients to prevent sliding on a 10° incline.

**Hint**: Start with `<mu>0.2</mu>` (robot slides down), incrementally increase to 0.8-1.0 until robot remains stationary. Measure angle at which sliding begins (static friction limit).

**Estimated Time**: 30 minutes

---

### Exercise 1.3: Compare ODE vs Bullet Performance (Advanced)
**Objective**: Spawn 10 boxes in Gazebo, measure real-time factor for ODE and Bullet engines, compare accuracy of contact forces.

**Hint**: Create two world files (one ODE, one Bullet) with identical parameters except `type="ode"` vs `type="bullet"`. Use Gazebo's contact sensor plugin to log forces, compare CPU usage and force magnitude variance.

**Estimated Time**: 45 minutes

---

## Summary

This chapter covered:
1. **Physics engine selection**: ODE for speed, Bullet for accuracy, DART for research
2. **Physics parameters**: Gravity (9.81 m/s²), timestep (0.001s), real-time factor (1.0), solver iterations (50)
3. **Contact properties**: Friction (0.8-1.0 for feet), restitution (0.0 for robots), stiffness/damping (kp=1e6, kd=1.0)
4. **ROS 2 integration**: `gazebo_ros_pkgs`, `spawn_entity.py`, topic remapping, launch file configuration
5. **Validation**: Robot falls at 9.81 m/s², contacts prevent penetration, real-time factor ≥0.5x

**Forward Reference**: In Chapter 2, you'll connect Unity to this physics simulation for photo-realistic visualization, enabling human-robot interaction scenarios with synchronized rendering.

---

## Further Reading

- **Koenig, N., & Howard, A. (2004)**. Design and use paradigms for Gazebo, an open-source multi-robot simulator. *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*. - Foundational paper describing Gazebo's architecture and ODE integration
- **Gazebo Classic Documentation**. Retrieved from https://classic.gazebosim.org/tutorials - Official tutorials on physics configuration, world files, and ROS 2 integration
- **Bullet Physics Manual**. Retrieved from https://github.com/bulletphysics/bullet3/blob/master/docs/Bullet_User_Manual.pdf - Deep dive into impulse-based solver used in Gazebo's Bullet engine

---

**Next Chapter**: [Chapter 2: Unity for High-Fidelity Simulation](./unity-visualization)
