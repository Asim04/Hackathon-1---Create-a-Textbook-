# Data Model: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature**: 002-digital-twin-gazebo-unity
**Created**: 2025-12-18
**Purpose**: Define content entities, attributes, relationships, and validation rules for Module 2

---

## Entity Definitions

### 1. Gazebo World

**Description**: Simulated environment file (.world or .sdf format) containing physics configuration, ground plane, lighting, obstacles, and robot spawn points.

**Attributes**:
- `name` (string): Unique world identifier (e.g., "humanoid_physics_world")
- `physics_engine` (enum): "ode" | "bullet" | "dart" (default: "ode")
- `gravity` (vector3): Gravitational acceleration in m/s² (default: [0, 0, -9.81])
- `max_step_size` (float): Physics timestep in seconds (default: 0.001, range: 0.0001-0.01)
- `real_time_factor` (float): Target simulation speed multiplier (default: 1.0, range: 0.1-2.0)
- `models` (array): List of included models (robots, obstacles, ground_plane, sun)

**Relationships**:
- CONTAINS 0-N Robot Models (defined by URDF/SDF)
- CONTAINS 0-N Obstacle Models (boxes, cylinders, meshes)
- INCLUDES 1 Ground Plane (collision surface)
- INCLUDES 0-N Light Sources (sun, point lights, spot lights)

**Example**:
```xml
<world name="digital_twin_world">
  <physics type="ode">
    <gravity>0 0 -9.81</gravity>
    <max_step_size>0.001</max_step_size>
  </physics>
  <include><uri>model://ground_plane</uri></include>
  <include><uri>model://sun</uri></include>
</world>
```

---

### 2. Unity Scene

**Description**: Unity project file (.unity) containing GameObjects, lighting, cameras, materials, and ROS communication components for photo-realistic robot visualization.

**Attributes**:
- `name` (string): Scene name (e.g., "DigitalTwinScene")
- `lighting_mode` (enum): "realtime" | "baked" | "mixed" (affects performance)
- `skybox` (asset): Background environment (procedural sky, HDRI, solid color)
- `camera_settings` (object): Main camera position, FOV, clipping planes
- `game_objects` (array): Hierarchy of objects (robot, environment, human avatars)
- `ros_connection` (component): ROSConnection GameObject with IP/port configuration

**Relationships**:
- CONTAINS 1 ROSConnection Component (manages ROS-TCP-Connector)
- CONTAINS 1-N Robot GameObjects (with Articulation Bodies)
- CONTAINS 0-N Environment Objects (floor, walls, obstacles)
- CONTAINS 0-N Human Avatars (for HRI visualization)
- INCLUDES 1-N Cameras (main, overhead, first-person views)
- INCLUDES 1-N Lights (directional, point, spot)

**Example** (C# Component):
```csharp
public class ROSConnection : MonoBehaviour {
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;
    // Manages TCP connection to ROS 2
}
```

---

### 3. Sensor Configuration

**Description**: Gazebo sensor plugin or Unity sensor component configuration defining sensor type, parameters, noise models, and ROS 2 topic publishing.

**Attributes**:
- `sensor_type` (enum): "lidar" | "depth_camera" | "rgb_camera" | "imu" | "contact" | "force_torque"
- `update_rate` (float): Sensor publish frequency in Hz (range: 1-100)
- `topic_name` (string): ROS 2 topic for sensor data (e.g., "/scan", "/camera/depth/image_raw")
- `frame_id` (string): TF frame for sensor coordinate system (e.g., "lidar_link")
- `noise_model` (object): Type ("gaussian"), mean, stddev for measurement noise
- `range` (object): min_range, max_range for distance sensors (meters)
- `resolution` (object): Width/height for cameras, angular resolution for LiDAR

**Relationships**:
- ATTACHED_TO Robot Link (via URDF <gazebo> tag or Unity parent transform)
- PUBLISHES_TO ROS 2 Topic (sensor_msgs/LaserScan, Image, Imu, etc.)
- VISUALIZED_IN RViz (using display plugins)

**Example** (Gazebo LiDAR Plugin):
```xml
<sensor name="lidar" type="ray">
  <update_rate>10.0</update_rate>
  <ray>
    <scan>
      <horizontal><samples>360</samples><resolution>1</resolution></horizontal>
      <vertical><samples>1</samples></vertical>
    </scan>
    <range><min>0.5</min><max>10.0</max></range>
  </ray>
  <plugin name="gazebo_ros_ray" filename="libgazebo_ros_ray_sensor.so">
    <ros><namespace>/robot</namespace></ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>lidar_link</frame_name>
  </plugin>
  <noise><type>gaussian</type><mean>0.0</mean><stddev>0.01</stddev></noise>
</sensor>
```

---

### 4. Physics Parameters

**Description**: Material and joint properties defining how robot components interact physically (masses, inertias, friction, damping).

**Attributes**:
- `mass` (float): Link mass in kilograms (range: 0.01-100 for humanoid components)
- `inertia_tensor` (matrix3x3): Moment of inertia (ixx, iyy, izz, ixy, ixz, iyz)
- `friction_coefficients` (object): mu1, mu2 for directional friction (range: 0.0-2.0)
- `damping` (float): Viscous damping coefficient (Nm/(rad/s) for revolute joints)
- `restitution` (float): Bounce coefficient (range: 0.0-1.0, typically 0.0 for robots)
- `contact_stiffness` (float): kp in Pascals (typically 1e6 for rigid bodies)
- `contact_damping` (float): kd damping coefficient (typically 1.0)

**Relationships**:
- APPLIES_TO Link (in URDF <inertial> tag or SDF <inertial>)
- APPLIES_TO Joint (in URDF <dynamics> tag)
- AFFECTS_SIMULATION Physics Engine Behavior

**Validation Rules**:
- Mass > 0 (physical constraint)
- Inertia tensor positive semi-definite (mathematical constraint)
- Friction coefficients ≥ 0 (physical constraint)
- Values should match real-world materials when possible (cite datasheets)

---

### 5. ROS-Unity Message

**Description**: Data structure exchanged between ROS 2 and Unity via TCP socket, serialized as JSON.

**Attributes**:
- `message_type` (string): ROS 2 message type (e.g., "sensor_msgs/JointState")
- `topic_name` (string): ROS 2 topic (e.g., "/joint_states")
- `direction` (enum): "ros_to_unity" | "unity_to_ros" | "bidirectional"
- `serialization_format` (enum): "json" (default for ROS-TCP-Connector)
- `update_rate` (float): Expected publish frequency (Hz)
- `latency_target` (float): Maximum acceptable latency in milliseconds (target: <50ms for SC-002)

**Relationships**:
- PUBLISHED_BY ROS 2 Node or Unity GameObject
- SUBSCRIBED_BY ROS 2 Node or Unity GameObject
- PART_OF ROS-Unity Bridge

**Example** (C# Subscriber):
```csharp
ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>(
    "/joint_states",
    UpdateRobotJoints
);
```

---

### 6. Digital Twin System

**Description**: Complete integrated system combining Gazebo (physics), Unity (visualization), ROS 2 (middleware), sensors, and control agents for robot simulation and testing.

**Attributes**:
- `system_name` (string): Identifier (e.g., "humanoid_digital_twin")
- `gazebo_world` (GazeboWorld): Physics simulation environment
- `unity_scene` (UnityScene): Visualization environment
- `ros_namespace` (string): ROS 2 namespace for topic isolation (e.g., "/robot")
- `sync_mode` (enum): "realtime" | "faster_than_realtime" | "slower_than_realtime"
- `components` (array): List of active components (perception agents, control agents, sensors)

**Relationships**:
- INTEGRATES Gazebo World + Unity Scene + ROS 2 Nodes
- SYNCHRONIZES State via ROS 2 Topics (/joint_states, /sensor_data)
- EXECUTES Control Loop (sensor → perception → decision → actuation)

**Data Flow**:
```
Gazebo (Physics Simulation)
    ↓ /joint_states, /sensor_data
ROS 2 Middleware (DDS)
    ↓ /joint_states
Unity (Visualization)
    ↑ (optional) /user_input, /camera_pose
ROS 2 Middleware
    ↓ /joint_commands
Gazebo (Actuator Control)
```

---

## Validation Rules

### Content Validation

- **Word count**: 1,200-1,500 per chapter (strict), 4,800-6,000 per module
- **Readability**: Flesch-Kincaid grade 10-12 (verify with online tool)
- **Citations**: Minimum 3 inline citations per chapter, 15+ total for module
- **Code examples**: All examples MUST execute successfully on target platform (Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11 + Unity 2021.3)

### Technical Validation

- **Gazebo worlds**: MUST load without errors, physics stable (no explosions/jittering), real-time factor ≥0.5x
- **Unity scenes**: MUST connect to ROS 2, frame rate ≥30 FPS, latency <50ms
- **Sensor configs**: MUST publish to correct topic, data format matches ROS 2 message spec, noise characteristics match specification
- **Digital twin**: MUST synchronize Gazebo and Unity states, reproducible behavior (same initial conditions → same outcomes ±5%)

### Educational Validation

- **Learning objectives**: Each chapter MUST have 2-3 specific, measurable objectives
- **Practice exercises**: 2-3 exercises per chapter (beginner to advanced)
- **Prerequisites**: Clearly state Module 1 knowledge and software requirements
- **Estimated time**: 12-15 hours total for module completion

---

**Data Model Version**: 1.0.0
**Last Updated**: 2025-12-18
**Entities**: 6 (Gazebo World, Unity Scene, Sensor Config, Physics Params, ROS-Unity Message, Digital Twin System)
**Ready for Content Creation**: ✅ Yes - Use as reference when writing chapters and creating code examples
