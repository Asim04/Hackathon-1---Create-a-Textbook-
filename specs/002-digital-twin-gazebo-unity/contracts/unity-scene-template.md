# Unity Scene Template: Module 2 - The Digital Twin

**Purpose**: Standard Unity 2021.3 LTS scene setup with ROS-TCP-Connector for robot visualization
**Version**: 1.0.0
**Compatible**: Unity 2021.3 LTS, ROS-TCP-Connector v0.7.0+, ROS 2 Humble
**Target Platform**: Windows, Linux, macOS

---

## Prerequisites

**Software Requirements**:
- Unity Hub 3.x installed ([download](https://unity.com/download))
- Unity 2021.3 LTS installed via Unity Hub
- ROS 2 Humble installed and sourced
- Git (for package installation)

**Knowledge Prerequisites**:
- Basic Unity navigation (Scene/Game view, Inspector, Hierarchy)
- Understanding of GameObjects and Components
- ROS 2 topics and message types (Module 1, Chapter 1)

---

## Step 1: Create Unity Project

1. **Open Unity Hub** and click **New Project**
2. **Select Template**: 3D (Core) - for standard rendering pipeline
3. **Project Settings**:
   - Project Name: `DigitalTwinSimulation`
   - Location: Choose directory (avoid spaces in path)
   - Unity Version: 2021.3.x LTS
4. **Create Project** (initial setup takes 2-3 minutes)

**Expected Result**: Unity Editor opens with empty scene containing Main Camera and Directional Light

---

## Step 2: Import ROS-TCP-Connector Package

### Method A: Git URL (Recommended)

1. In Unity Editor, go to **Window > Package Manager**
2. Click **+ (Plus)** > **Add package from git URL**
3. Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
4. Click **Add** (installation takes 30-60 seconds)

### Method B: Manual Installation

1. Clone repository:
   ```bash
   cd ~/Downloads
   git clone https://github.com/Unity-Technologies/ROS-TCP-Connector.git
   ```
2. In Unity: **Window > Package Manager > + > Add package from disk**
3. Navigate to `ROS-TCP-Connector/com.unity.robotics.ros-tcp-connector/package.json`

**Verification**: Check **Window > Package Manager > Packages: In Project** - should see "ROS TCP Connector"

---

## Step 3: Configure ROS Connection

1. **Create ROSConnectionPrefab**:
   - **Robotics > ROS Settings** (opens Inspector)
   - Set **ROS IP Address**: `127.0.0.1` (localhost for same-machine setup)
   - Set **ROS Port**: `10000` (default ROS-TCP-Endpoint port)
   - Protocol: **ROS 2** (dropdown)

2. **Add to Scene**:
   - In Hierarchy, **Right-click > Create Empty**
   - Rename to `ROSConnection`
   - **Add Component > ROS Connection** (type in search bar)
   - Inspector should show ROS Connection component with IP/Port fields

3. **Verify Settings**:
   ```
   IP Address: 127.0.0.1
   Port: 10000
   Protocol: ROS 2
   Show HUD: ✓ (for debugging)
   ```

**Note**: For remote ROS machines, replace `127.0.0.1` with ROS machine's IP address

---

## Step 4: Set Up Robot Visualization

### Option A: Using URDF Importer (Recommended)

1. **Install URDF Importer**:
   - **Window > Package Manager > + > Add package from git URL**
   - Enter: `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`

2. **Import Robot URDF**:
   - **Assets > Import Robot from URDF** (or **Robotics > Import Robot from URDF**)
   - Select your robot's `.urdf` file (e.g., `humanoid_robot.urdf`)
   - Import Settings:
     - Axis Type: **Z Axis (ROS 2 Default)**
     - Mesh Decomposer: **VHACD** (for collision meshes)
   - Click **Import**

3. **Expected Result**: Robot appears in Hierarchy as GameObject tree matching URDF link structure

### Option B: Manual GameObject Hierarchy

1. **Create Root GameObject**:
   - Hierarchy > **Right-click > 3D Object > Cube** (placeholder for base link)
   - Rename to `robot_base`
   - Position: `(0, 0.5, 0)` (half-height above ground)

2. **Add Child Links**:
   - Right-click `robot_base` > **3D Object > Cylinder** (for limbs)
   - Rename to `right_arm_link`
   - Adjust scale/position to match robot morphology

3. **Add Articulation Body** (for physics-based joints):
   - Select `robot_base` > **Add Component > Articulation Body**
   - Set **Immovable**: ✓ (for base link)
   - For child links: **Add Component > Articulation Body** with **Joint Type: Revolute/Prismatic**

---

## Step 5: Configure Lighting and Post-Processing

### Lighting Setup

1. **Select Directional Light** in Hierarchy
2. Inspector settings:
   - Intensity: `1.0` (default)
   - Color: `#FFFFFF` (white)
   - Shadow Type: **Soft Shadows** (for realism)
   - Transform Rotation: `(50, -30, 0)` (top-down angle)

3. **Add Environment Lighting**:
   - **Window > Rendering > Lighting**
   - Environment tab:
     - Skybox Material: Default-Skybox (or custom HDRI)
     - Ambient Source: **Skybox**
     - Ambient Intensity: `1.0`

### Post-Processing (Optional for Photo-Realism)

1. **Install Post Processing Package**:
   - **Window > Package Manager** > Unity Registry
   - Search "Post Processing" > **Install**

2. **Add Post-Process Volume**:
   - Hierarchy > **Right-click > Volume > Global Volume**
   - Inspector: **Add Override > Bloom** (glow effect)
   - Add Override > **Ambient Occlusion** (contact shadows)

**Performance Note**: Post-processing reduces frame rate by 10-20%; disable for low-end hardware

---

## Step 6: Create Joint State Subscriber Script

### C# Script: RobotJointSubscriber.cs

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

/// <summary>
/// Subscribes to /joint_states and updates Unity Articulation Bodies
/// </summary>
public class RobotJointSubscriber : MonoBehaviour
{
    // ROS 2 topic to subscribe to
    public string jointStateTopic = "/joint_states";

    // Reference to robot's root Articulation Body
    public ArticulationBody robotRoot;

    private ArticulationBody[] joints;

    void Start()
    {
        // Get all Articulation Bodies in robot hierarchy
        joints = robotRoot.GetComponentsInChildren<ArticulationBody>();

        // Subscribe to ROS 2 topic
        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>(
            jointStateTopic,
            UpdateJointStates
        );

        Debug.Log($"Subscribed to {jointStateTopic}");
    }

    /// <summary>
    /// Callback when new joint state message arrives
    /// </summary>
    void UpdateJointStates(JointStateMsg jointStateMsg)
    {
        // Match ROS joint names to Unity Articulation Bodies
        for (int i = 0; i < jointStateMsg.name.Length; i++)
        {
            string jointName = jointStateMsg.name[i];
            double jointPosition = jointStateMsg.position[i];

            // Find matching Unity joint
            foreach (var joint in joints)
            {
                if (joint.name == jointName)
                {
                    // Set target position (in radians for revolute)
                    var drive = joint.xDrive;
                    drive.target = (float)(jointPosition * Mathf.Rad2Deg);
                    joint.xDrive = drive;
                    break;
                }
            }
        }
    }
}
```

### Attach Script to Robot

1. **Create Script**:
   - **Assets > Create > C# Script** > Name: `RobotJointSubscriber`
   - Double-click to open in IDE, paste code above, save

2. **Attach to GameObject**:
   - Select robot root GameObject in Hierarchy
   - **Add Component** > type "RobotJointSubscriber"
   - Drag robot root into **Robot Root** field in Inspector
   - Set **Joint State Topic**: `/joint_states` (or your custom topic)

**Expected Behavior**: When ROS publishes to `/joint_states`, robot joints move in Unity

---

## Step 7: Test ROS-Unity Connection

### Terminal 1: Start ROS-TCP-Endpoint

```bash
# Source ROS 2 workspace
source /opt/ros/humble/setup.bash

# Install endpoint (first time only)
pip3 install ros-tcp-endpoint

# Run endpoint server
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**Expected Output**:
```
[INFO] [ros_tcp_endpoint]: ROS-TCP Endpoint started on 0.0.0.0:10000
```

### Terminal 2: Publish Test Joint States

```bash
source /opt/ros/humble/setup.bash

# Publish single joint state message
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{
  name: ['right_arm_joint'],
  position: [1.57]
}" --once
```

### Unity Editor

1. **Press Play** (top center button)
2. **Check Console** (Window > General > Console):
   - Should see: `Subscribed to /joint_states`
   - HUD overlay shows connection status (green = connected)
3. **Observe Robot**: Joint should move to 1.57 radians (~90 degrees)

**Troubleshooting**:
- If "Connection failed": Check firewall allows port 10000
- If robot doesn't move: Verify joint names match between ROS and Unity
- If Console shows errors: Check ROS 2 is sourced in same terminal as endpoint

---

## Step 8: Save Scene

1. **File > Save As**
2. Name: `DigitalTwinScene.unity`
3. Location: `Assets/Scenes/` (create folder if needed)

---

## Common Issues and Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| "Package ROS-TCP-Connector not found" | Git URL incorrect or network issue | Verify URL matches GitHub repo, check internet connection |
| Robot mesh appears pink | Missing materials/textures | Re-import URDF with "Import Materials" checked |
| Unity freezes on Play | Infinite loop in Update() | Check custom scripts for while loops, remove or add yield |
| Joint positions inverted | Axis convention mismatch | Multiply joint angles by -1 in UpdateJointStates() |
| Connection timeout | ROS endpoint not running | Start `ros_tcp_endpoint` before pressing Play in Unity |

---

## Performance Optimization Tips

- **Reduce Polygon Count**: Use simplified meshes for real-time visualization (< 10k triangles/model)
- **Disable Shadows**: For non-essential objects, Inspector > Mesh Renderer > Cast Shadows: Off
- **Occlusion Culling**: Window > Rendering > Occlusion Culling (bake for static environments)
- **Target Frame Rate**: Edit > Project Settings > Quality > VSync Count: Every V Blank (caps at 60 FPS)

---

## Next Steps

- **Chapter 2**: Integrate this Unity scene with Gazebo simulation for synchronized physics + visualization
- **Chapter 3**: Add sensor visualizations (LiDAR point clouds, depth camera outputs)
- **Chapter 4**: Complete digital twin with bidirectional control

---

**Template Version**: 1.0.0
**Last Updated**: 2025-12-18
**Validated On**: Unity 2021.3.16f1, ROS 2 Humble, Ubuntu 22.04
**Estimated Setup Time**: 30-45 minutes (first time), 10-15 minutes (subsequent)
