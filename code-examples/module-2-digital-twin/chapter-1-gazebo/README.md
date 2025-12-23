# Chapter 1: Gazebo Physics Simulation - Code Examples

**Module**: The Digital Twin (Gazebo & Unity)
**Chapter**: 1 - Gazebo Physics Simulation
**Purpose**: Demonstrate physics configuration (ODE, gravity, contact properties) and ROS 2 integration

---

## Files in this Directory

1. **`basic_world.world`**: Minimal Gazebo world with ODE physics configuration (gravity 9.81 m/s², timestep 0.001s)
2. **`humanoid_physics.world`**: World with test box and high-friction ground plane for humanoid simulation
3. **`launch_gazebo.launch.py`**: ROS 2 launch file to start Gazebo, spawn robot, and publish joint states

---

## Prerequisites

**Software**:
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Gazebo Classic 11

**Installation**:
```bash
# Install Gazebo ROS 2 packages (if not already installed)
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-joint-state-publisher ros-humble-robot-state-publisher -y
```

**Verification**:
```bash
# Check Gazebo version
gazebo --version
# Expected: Gazebo multi-robot simulator, version 11.x.x

# Check ROS 2 packages
ros2 pkg list | grep gazebo
# Expected: gazebo_ros, gazebo_ros_pkgs, gazebo_plugins, etc.
```

---

## Example 1: Basic World

### Purpose
Validate physics configuration (gravity, timestep, solver settings) with minimal setup.

### How to Run

```bash
# Navigate to this directory
cd code-examples/module-2-digital-twin/chapter-1-gazebo/

# Launch Gazebo with basic world
gazebo basic_world.world
```

### Expected Output

- **Gazebo GUI** opens showing:
  - Gray ground plane
  - Directional sunlight casting shadows
  - Empty 3D world (no models yet)
- **Physics Info** (bottom right of GUI):
  - Real-time factor: ~1.0 (simulation runs at real-time speed)
  - Step size: 0.001s (1 kHz update rate)

### What to Observe

1. **Camera Controls**: Left-click drag to rotate, middle-click drag to pan, scroll to zoom
2. **Simulation Time**: Bottom left shows elapsed time (t=0.0s at start, increments at 1.0s/s)
3. **Performance**: Real-time factor should remain close to 1.0 (if drops below 0.5, system is underpowered for this physics config)

---

## Example 2: Humanoid Physics World

### Purpose
Demonstrate contact properties (friction, restitution, stiffness) with falling test box.

### How to Run

```bash
# Launch Gazebo with humanoid physics world
gazebo humanoid_physics.world
```

### Expected Output

- **Gazebo GUI** opens with:
  - Ground plane (gray, high friction)
  - **Red test box** at position (0, 0, 1.0) - **immediately falls due to gravity**
- **Physics Behavior**:
  - Box falls for ~0.45 seconds (validates gravity = 9.81 m/s²)
  - Box hits ground with **no bounce** (restitution = 0.0)
  - Box **does not penetrate** ground (rigid contact: kp=1e6)
  - Box **does not slide** when ground is tilted (high friction: mu=1.0)

### Validation Tests

#### Test 1: Falling Time
**Theoretical**: For 1 meter drop, time = sqrt(2h/g) = sqrt(2*1/9.81) ≈ 0.45s

**Procedure**:
1. Note simulation time when box starts falling (t=0.0s)
2. Note time when box contacts ground (check for contact force in View > Contacts)
3. Measured time should be 0.43-0.47s (within numerical error)

#### Test 2: No Bounce
**Expected**: Box remains stationary after contact (restitution = 0.0)

**Procedure**:
1. After box lands, observe position (should be z ≈ 0.5m - half box height)
2. Box should not bounce back up
3. If bouncing: Check `<restitution_coefficient>` in URDF

#### Test 3: Friction
**Expected**: Box does not slide on tilted ground (mu=1.0)

**Procedure**:
1. Right-click box > "Apply Force" > Z-axis: 10N (pushes sideways)
2. Box should remain in place or move slowly (high friction prevents sliding)
3. Reduce friction to 0.2 in world file, reload - box should slide easily

---

## Example 3: ROS 2 Launch File

### Purpose
Launch Gazebo from ROS 2, spawn humanoid robot, publish joint states for ROS 2 integration.

### Setup

#### Option A: Standalone Execution (No ROS 2 Package)

1. **Edit `launch_gazebo.launch.py`** - Update paths:
   ```python
   world_file = '/absolute/path/to/humanoid_physics.world'
   robot_urdf_path = '/absolute/path/to/humanoid.urdf'  # From Module 1
   ```

2. **Run Directly**:
   ```bash
   python3 launch_gazebo.launch.py
   # Note: This will fail without proper ROS 2 launch setup
   ```

#### Option B: ROS 2 Package Setup (Recommended)

1. **Create ROS 2 Package**:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create module_2_gazebo_examples --build-type ament_python
   cd module_2_gazebo_examples
   ```

2. **Create Directory Structure**:
   ```bash
   mkdir -p launch worlds urdf
   cp /path/to/chapter-1-gazebo/*.world worlds/
   cp /path/to/chapter-1-gazebo/*.launch.py launch/
   cp /path/to/module-1/humanoid.urdf urdf/
   ```

3. **Update `setup.py`**:
   ```python
   from setuptools import setup
   import os
   from glob import glob

   setup(
       name='module_2_gazebo_examples',
       version='1.0.0',
       packages=['module_2_gazebo_examples'],
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/module_2_gazebo_examples']),
           ('share/module_2_gazebo_examples', ['package.xml']),
           ('share/module_2_gazebo_examples/launch', glob('launch/*.launch.py')),
           ('share/module_2_gazebo_examples/worlds', glob('worlds/*.world')),
           ('share/module_2_gazebo_examples/urdf', glob('urdf/*.urdf')),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
   )
   ```

4. **Build and Source**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select module_2_gazebo_examples
   source install/setup.bash
   ```

5. **Launch**:
   ```bash
   ros2 launch module_2_gazebo_examples launch_gazebo.launch.py
   ```

### Expected Output

**Terminal**:
```
[INFO] [launch]: All log files can be found below /home/user/.ros/log/...
[INFO] [gazebo-1]: process started with pid [12345]
[INFO] [spawn_entity.py-2]: process started with pid [12346]
[INFO] [joint_state_publisher-3]: process started with pid [12347]
[INFO] [robot_state_publisher-4]: process started with pid [12348]
[gazebo-1] Gazebo multi-robot simulator, version 11.x.x
[spawn_entity.py-2] [INFO] [spawn_entity]: Waiting for service /spawn_entity
[spawn_entity.py-2] [INFO] [spawn_entity]: Spawn status: SpawnEntity: successfully spawned entity 'humanoid_robot'
```

**Gazebo GUI**:
- Opens with humanoid physics world
- Humanoid robot spawns at (0, 0, 1.0) after ~2 seconds
- Robot falls and collides with ground

**ROS 2 Topics**:
```bash
# Check published topics
ros2 topic list
# Expected output includes:
# /clock
# /joint_states
# /gazebo/link_states
# /tf
# /tf_static

# Monitor joint states
ros2 topic echo /joint_states
# Should show joint positions/velocities updating at ~1 kHz
```

---

## Troubleshooting

### Issue 1: "Gazebo: command not found"

**Cause**: Gazebo Classic 11 not installed or not in PATH

**Solution**:
```bash
sudo apt install gazebo11 -y
# Verify installation
gazebo --version
```

---

### Issue 2: Model not found / Spawn failed

**Cause**: Incorrect path to URDF in launch file or humanoid.urdf missing

**Solution**:
1. Verify URDF path:
   ```bash
   ls -lh /path/to/humanoid.urdf
   ```
2. Update `robot_urdf_path` in `launch_gazebo.launch.py` to correct absolute path
3. Ensure URDF is valid XML:
   ```bash
   check_urdf humanoid.urdf
   # Expected: robot name is: humanoid
   ```

---

### Issue 3: Physics unstable (robot explodes/jitters)

**Cause**: Timestep too large or insufficient solver iterations

**Solution**:
1. Open `humanoid_physics.world` in text editor
2. Reduce timestep:
   ```xml
   <max_step_size>0.0005</max_step_size>  <!-- Half the default -->
   ```
3. Increase solver iterations:
   ```xml
   <iters>100</iters>  <!-- Double the default -->
   ```
4. Reload world and re-spawn robot

---

### Issue 4: Real-time factor low (<0.5)

**Cause**: Physics computation exceeds available CPU, or complex collision meshes

**Solution**:
1. **Simplify collision meshes**: Replace detailed STL meshes with primitive shapes (boxes, cylinders)
2. **Increase timestep**: Change to 0.002s (less accurate but faster)
3. **Disable shadows**: In world file, set `<shadows>false</shadows>`
4. **Reduce GUI load**: Launch headless with `gzserver` only (no `gzclient` GUI):
   ```bash
   gzserver humanoid_physics.world
   ```

---

### Issue 5: Joint controller not working

**Cause**: `ros2_control` hardware interface not configured in URDF

**Solution**:
1. Ensure URDF has `<gazebo>` plugin for `gazebo_ros2_control`:
   ```xml
   <gazebo>
     <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
       <robot_sim_type>gazebo_ros2_control/GazeboSystem</robot_sim_type>
     </plugin>
   </gazebo>
   ```
2. Add `<ros2_control>` block with joint interfaces (see Module 3 for details)

---

## Further Experiments

### Experiment 1: Moon Gravity
**Objective**: Simulate lunar gravity (1.62 m/s²) and measure falling time

**Procedure**:
1. Edit `humanoid_physics.world`
2. Change `<gravity>0 0 -9.81</gravity>` to `<gravity>0 0 -1.62</gravity>`
3. Save and relaunch Gazebo
4. Measure falling time (should be ~1.1s for 1m drop vs 0.45s on Earth)

### Experiment 2: Friction Coefficient Sweep
**Objective**: Find minimum friction to prevent sliding on 10° incline

**Procedure**:
1. Edit ground plane `<mu>` values (0.1, 0.2, 0.4, 0.6, 0.8, 1.0)
2. For each value:
   - Tilt ground 10° (rotate ground plane model in Gazebo GUI)
   - Observe if box slides
3. Record critical friction coefficient (where sliding stops)
4. Compare to theoretical: mu_min = tan(10°) ≈ 0.176

### Experiment 3: ODE vs Bullet Comparison
**Objective**: Compare physics engine accuracy and performance

**Procedure**:
1. Duplicate `humanoid_physics.world` as `humanoid_bullet.world`
2. Change `<physics type="ode">` to `<physics type="bullet">`
3. Launch both worlds side-by-side
4. Measure:
   - Falling time (should be similar: ~0.45s)
   - Real-time factor (ODE typically faster)
   - Contact stability (Bullet less jitter on complex contacts)

---

**Next Steps**: Proceed to [Chapter 2: Unity for High-Fidelity Simulation](../../docs/module-2-digital-twin/02-unity-visualization.md) to connect Unity visualization to this physics simulation.

---

**Version**: 1.0.0
**Last Updated**: 2025-12-18
**Tested On**: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11.14.0
