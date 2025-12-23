# Unity Project Structure for Chapter 2

**Module**: The Digital Twin (Gazebo & Unity)
**Chapter**: 2 - Unity for High-Fidelity Simulation
**Unity Version**: 2021.3 LTS
**Purpose**: Standard project layout for digital twin visualization

---

## Directory Structure

```
DigitalTwinVisualization/              # Unity project root
├── Assets/
│   ├── Scenes/
│   │   ├── DigitalTwinScene.unity     # Main scene with robot + ROS connection
│   │   └── BasicScene.unity           # Empty scene for testing
│   ├── Scripts/
│   │   ├── RobotController.cs         # Subscribes to /joint_states, updates Articulation Bodies
│   │   ├── ROSConnection.cs           # Initializes ROS-TCP-Connector, handles errors
│   │   └── LatencyMonitor.cs          # (Optional) Measures ROS-Unity sync latency
│   ├── Prefabs/
│   │   ├── HumanoidRobot.prefab       # Robot GameObject hierarchy with Articulation Bodies
│   │   └── ROSConnectionPrefab.prefab # ROS connection settings (IP, port, protocol)
│   ├── Materials/
│   │   ├── RobotMetal.mat             # PBR material for robot body (metallic)
│   │   ├── RobotPlastic.mat           # PBR material for robot joints (smooth plastic)
│   │   └── Ground.mat                 # Ground plane material
│   ├── Models/
│   │   └── humanoid_mesh.fbx          # Robot 3D mesh (imported from URDF or CAD)
│   └── ROS/
│       └── MessageTypes/              # Generated C# classes for ROS 2 messages
│           ├── JointStateMsg.cs       # sensor_msgs/JointState
│           ├── TwistMsg.cs            # geometry_msgs/Twist
│           └── PoseMsg.cs             # geometry_msgs/Pose
├── Packages/
│   ├── manifest.json                  # Package dependencies
│   └── com.unity.robotics.ros-tcp-connector/  # ROS-TCP-Connector package
├── ProjectSettings/
│   ├── ProjectSettings.asset          # Unity version, target platforms
│   ├── QualitySettings.asset          # Graphics quality (VSync, anti-aliasing)
│   └── PackageManagerSettings.asset   # Package Manager configuration
├── Library/                            # Build cache (auto-generated, gitignored)
├── Logs/                               # Unity Editor logs (auto-generated)
├── Temp/                               # Temporary build files (auto-generated)
└── UserSettings/                       # Per-user Editor settings (auto-generated)
```

---

## Key Files and Purposes

### Assets/Scenes/DigitalTwinScene.unity
Main scene containing:
- **Ground Plane**: Quad mesh with Ground material, Transform position (0, 0, 0)
- **Directional Light**: Simulates sunlight, intensity 1.0, color white
- **Main Camera**: Position (5, 3, -5), looks at origin
- **HumanoidRobot**: Prefab instance at (0, 0, 0)
- **ROSConnection**: GameObject with ROSConnection component (IP: 127.0.0.1, port: 10000)

### Assets/Scripts/RobotController.cs
Responsibilities:
- Subscribe to `/joint_states` ROS 2 topic
- Parse `sensor_msgs/JointState` messages
- Update Articulation Body joint targets (xDrive.target)
- Handle joint name matching between ROS and Unity

### Assets/Scripts/ROSConnection.cs
Responsibilities:
- Initialize ROS-TCP-Connector on scene start
- Configure IP address, port, protocol (ROS 2)
- Display connection status in Unity HUD
- Handle connection errors (timeout, refused)

### Assets/Prefabs/HumanoidRobot.prefab
Hierarchy:
```
HumanoidRobot (root, Articulation Body: immovable)
├── torso_link (Articulation Body: fixed)
├── right_shoulder_link (Articulation Body: revolute)
│   └── right_upper_arm_link (Articulation Body: revolute)
│       └── right_lower_arm_link (Articulation Body: revolute)
│           └── right_hand_link (Articulation Body: fixed)
├── left_shoulder_link (Articulation Body: revolute)
│   └── ... (similar to right arm)
├── right_hip_link (Articulation Body: revolute)
│   └── right_upper_leg_link (Articulation Body: revolute)
│       └── right_lower_leg_link (Articulation Body: revolute)
│           └── right_foot_link (Articulation Body: fixed)
└── left_hip_link (Articulation Body: revolute)
    └── ... (similar to right leg)
```

Each Articulation Body configured with:
- `jointType`: RevoluteJoint or PrismaticJoint
- `xDrive.stiffness`: 10000 (moderate spring force)
- `xDrive.damping`: 100 (light damping for smooth motion)
- `xDrive.forceLimit`: 1000 (torque limit in Newton-meters)

---

## Package Dependencies

### Packages/manifest.json
```json
{
  "dependencies": {
    "com.unity.robotics.ros-tcp-connector": "https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector",
    "com.unity.render-pipelines.universal": "12.1.7",
    "com.unity.textmeshpro": "3.0.6",
    "com.unity.ide.visualstudio": "2.0.16",
    "com.unity.ide.rider": "3.0.15",
    "com.unity.test-framework": "1.1.31"
  }
}
```

**Key Packages**:
- `ros-tcp-connector`: ROS 2 communication (required)
- `render-pipelines.universal`: URP for photo-realistic rendering (recommended)
- `textmeshpro`: UI text for connection status HUD
- `test-framework`: Unit testing for C# scripts

---

## Project Settings

### ProjectSettings/QualitySettings.asset
Recommended settings for digital twin performance:

```yaml
QualitySettings:
  m_QualitySettings:
    - name: Medium
      pixelLightCount: 2
      shadows: SoftShadows
      shadowResolution: MediumResolution
      antiAliasing: 2          # 2x MSAA for smoother edges
      vSyncCount: 1            # Enable VSync (60 FPS target)
      anisotropicTextures: PerTexture
```

**Rationale**:
- VSync: Locks to 60 FPS (16.67ms frame time) for <50ms latency budget
- 2x MSAA: Reduces aliasing without heavy performance cost
- Soft shadows: Better visual quality for robot demos

### ProjectSettings/ProjectSettings.asset
```yaml
PlayerSettings:
  productName: Digital Twin Visualization
  companyName: Your Organization
  defaultScreenWidth: 1920
  defaultScreenHeight: 1080
  targetFrameRate: 60        # Force 60 FPS even without VSync
  runInBackground: true      # Continue simulation when Unity window unfocused
```

---

## .gitignore Recommendations

Add to `.gitignore` at project root:

```gitignore
# Unity generated folders
[Ll]ibrary/
[Tt]emp/
[Oo]bj/
[Bb]uild/
[Bb]uilds/
[Ll]ogs/
[Uu]ser[Ss]ettings/

# Visual Studio cache
.vs/
*.csproj
*.unityproj
*.sln
*.suo
*.user
*.userprefs

# OS generated
.DS_Store
.DS_Store?
._*
.Spotlight-V100
.Trashes
ehthumbs.db
Thumbs.db

# Unity crash logs
sysinfo.txt
*.stackdump
```

---

## Setup Instructions

### 1. Create Project in Unity Hub
```
Unity Hub > New Project
Template: 3D (Core) or 3D (URP)
Project Name: DigitalTwinVisualization
Location: code-examples/module-2-digital-twin/chapter-2-unity/
Unity Version: 2021.3.x LTS
```

### 2. Import ROS-TCP-Connector
```
Unity Editor > Window > Package Manager
+ (Plus) > Add package from git URL
URL: https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
```

### 3. Create Directory Structure
In Unity Project window:
```
Right-click Assets > Create > Folder
Create: Scripts, Scenes, Prefabs, Materials, Models, ROS
```

### 4. Configure Quality Settings
```
Edit > Project Settings > Quality
Select "Medium" preset
Enable VSync (VSync Count: Every V Blank)
Anti Aliasing: 2x Multi Sampling
```

### 5. Add Scripts
Copy `RobotController.cs` and `ROSConnection.cs` to `Assets/Scripts/`

### 6. Create Scene
```
File > New Scene
Save as: Assets/Scenes/DigitalTwinScene.unity
Add: Ground plane (3D Object > Quad, Scale: 10x10)
Add: Directional Light (default)
Add: Empty GameObject "ROSConnection"
```

---

## Testing Project Structure

### Verification Checklist
- [ ] Unity 2021.3 LTS opens project without errors
- [ ] ROS-TCP-Connector visible in Package Manager (In Project)
- [ ] Folder structure matches layout above
- [ ] Quality settings: VSync enabled, 2x MSAA
- [ ] DigitalTwinScene.unity opens in Scene view
- [ ] Console shows no errors on project load

### Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| "Package not found" error | Git URL incorrect | Verify URL matches GitHub repository exactly |
| Unity crashes on load | Incompatible Unity version | Ensure 2021.3.x LTS (not 2022.x or 2020.x) |
| Scripts missing after import | Wrong folder location | Scripts must be in `Assets/Scripts/` |
| Low frame rate | Quality settings too high | Reduce shadow resolution, disable post-processing |

---

## Next Steps

1. Implement `RobotController.cs` (see Chapter 2, Section 4)
2. Implement `ROSConnection.cs` (see Chapter 2, Section 2)
3. Create HumanoidRobot prefab with Articulation Bodies
4. Test connection to ROS 2 (Chapter 2, Exercise 2.1)

---

**Project Structure Version**: 1.0.0
**Last Updated**: 2025-12-18
**Compatible With**: Unity 2021.3 LTS, ROS 2 Humble, ROS-TCP-Connector v0.7.0+
