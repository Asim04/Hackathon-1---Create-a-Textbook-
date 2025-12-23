# Basic Unity Scene Setup

**Purpose**: Step-by-step guide to create a minimal Unity scene for testing ROS 2 connection
**Scene Name**: BasicScene.unity
**Estimated Time**: 15 minutes

---

## Scene Components

### 1. Ground Plane
- **Type**: 3D Object > Quad
- **Transform**:
  - Position: (0, 0, 0)
  - Rotation: (90, 0, 0) - Lays flat horizontally
  - Scale: (10, 10, 1) - 10m × 10m ground surface
- **Material**: Ground.mat (gray, slightly reflective)
- **Purpose**: Provides visual reference plane for robot placement

**Creation Steps**:
```
1. GameObject > 3D Object > Quad
2. Rename to "GroundPlane"
3. Inspector > Transform:
   - Position X: 0, Y: 0, Z: 0
   - Rotation X: 90, Y: 0, Z: 0
   - Scale X: 10, Y: 10, Z: 1
4. Drag "Ground.mat" from Assets/Materials/ onto GroundPlane in Scene view
```

---

### 2. Directional Light
- **Type**: Light > Directional Light (simulates sun)
- **Transform**:
  - Position: (0, 3, 0) - Position doesn't affect directional lights
  - Rotation: (50, -30, 0) - Top-down angle casting shadows
- **Light Settings**:
  - Color: White (255, 255, 255)
  - Intensity: 1.0
  - Shadow Type: Soft Shadows
  - Shadow Resolution: Medium Resolution
- **Purpose**: Illuminates scene, casts shadows for depth perception

**Creation Steps**:
```
1. GameObject > Light > Directional Light
2. Rename to "Sun"
3. Inspector > Light Component:
   - Color: RGB(255, 255, 255)
   - Intensity: 1.0
   - Mode: Realtime
   - Shadow Type: Soft Shadows
4. Inspector > Transform > Rotation:
   - X: 50, Y: -30, Z: 0
```

---

### 3. Main Camera
- **Type**: Camera (auto-created with new scene)
- **Transform**:
  - Position: (5, 3, -5) - Angled view of origin
  - Rotation: (20, -45, 0) - Looks toward (0, 0, 0)
- **Camera Settings**:
  - Projection: Perspective
  - Field of View: 60°
  - Clipping Planes: Near 0.3, Far 1000
  - Target Display: Display 1
- **Purpose**: Renders scene from player viewpoint

**Creation Steps**:
```
1. Select "Main Camera" in Hierarchy (exists by default)
2. Inspector > Transform:
   - Position X: 5, Y: 3, Z: -5
   - Rotation X: 20, Y: -45, Z: 0
3. Verify Camera component settings:
   - Projection: Perspective
   - FOV: 60
4. Test view: Scene view > Align View to Selected (Ctrl+Shift+F with camera selected)
```

**Camera View Preview**:
When positioned correctly, camera should show:
- Ground plane filling lower half of view
- Origin (0, 0, 0) centered in frame
- Directional light casting shadows diagonally across ground

---

### 4. Skybox (Optional)
- **Type**: Lighting > Environment
- **Skybox Material**: Default-Skybox (blue gradient)
- **Ambient Source**: Skybox
- **Ambient Intensity**: 1.0
- **Purpose**: Provides background color and ambient lighting

**Configuration Steps**:
```
1. Window > Rendering > Lighting
2. Environment tab
3. Skybox Material: Default-Skybox (or drag custom skybox)
4. Environment Lighting:
   - Source: Skybox
   - Intensity Multiplier: 1.0
5. (Optional) Adjust Sun Source to "Directional Light" for dynamic shadows
```

---

### 5. ROSConnection GameObject
- **Type**: Empty GameObject
- **Transform**: Position (0, 0, 0) - Position irrelevant, no visual representation
- **Components**:
  - ROSConnection (custom script from ROS-TCP-Connector)
  - Settings:
    - ROS IP Address: 127.0.0.1 (localhost)
    - ROS Port: 10000
    - Protocol: ROS 2
    - Show HUD: ✓ (displays connection status overlay)
- **Purpose**: Manages TCP connection to ROS-TCP-Endpoint

**Creation Steps**:
```
1. GameObject > Create Empty
2. Rename to "ROSConnection"
3. Inspector > Add Component > Search "ROS Connection"
4. Configure ROS Connection component:
   - ROS IP Address: 127.0.0.1
   - ROS Port: 10000
   - Protocol: ROS 2
   - Show HUD: Check this box
5. (Optional) Tag as "ROS" for easy GameObject.FindWithTag() access
```

---

## Scene Hierarchy

Final hierarchy in Unity Hierarchy window:

```
BasicScene
├── Main Camera
├── Directional Light (Sun)
├── GroundPlane (Quad)
└── ROSConnection (Empty GameObject)
```

---

## Visual Appearance

When scene is complete, Game view should display:

- **Background**: Sky blue gradient (default skybox)
- **Ground**: Gray 10m × 10m plane with soft shadows
- **Lighting**: Bright, natural outdoor lighting from top-right
- **HUD Overlay** (top-left corner):
  ```
  ROS IP: 127.0.0.1:10000
  Status: Connecting... (yellow)
  ```
  (Changes to "Connected" in green when ROS-TCP-Endpoint is running)

---

## Testing the Scene

### 1. Play Mode Test (No ROS)
```
1. Press Play button (top center of Unity Editor)
2. Game view displays scene
3. HUD shows "Status: Connecting..." (yellow) - Expected, ROS not running yet
4. Console shows: "ROSConnection: Attempting to connect to 127.0.0.1:10000"
5. Press Play again to stop
```

**Expected Behavior**: Scene renders correctly, HUD appears, no errors in Console

### 2. Connection Test (With ROS)

**Terminal 1 - Start ROS-TCP-Endpoint**:
```bash
source /opt/ros/humble/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
# Expected output: [INFO] [ros_tcp_endpoint]: ROS-TCP Endpoint started on 0.0.0.0:10000
```

**Unity Editor**:
```
1. Press Play
2. HUD should change to "Status: Connected" (green) within 2-3 seconds
3. Console shows: "ROSConnection: Connected to ROS-TCP-Endpoint"
4. Test successful!
```

### 3. Frame Rate Test
```
1. Window > Analysis > Profiler
2. Press Play
3. Check FPS in Profiler (should be 60 FPS with VSync enabled)
4. If FPS < 30, reduce Quality settings or shadow resolution
```

---

## Common Setup Issues

| Issue | Symptoms | Solution |
|-------|----------|----------|
| Ground plane not visible | Scene view shows blue sky only | Check Quad rotation is (90, 0, 0), not (0, 0, 0) |
| Camera shows nothing | Game view is black or blue | Verify camera position (5, 3, -5) and rotation (20, -45, 0) |
| HUD doesn't appear | No connection status overlay | Enable "Show HUD" in ROS Connection component |
| Connection always yellow | "Connecting..." never turns green | Check ROS-TCP-Endpoint is running on port 10000 |
| Dark scene | Everything appears black/gray | Check Directional Light intensity is 1.0, not 0.0 |
| Low frame rate | Game view stutters, FPS < 30 | Disable shadows or reduce Quality settings to Low |

---

## Extending the Basic Scene

### Add a Test Cube (for visual reference)
```
1. GameObject > 3D Object > Cube
2. Position: (0, 0.5, 0) - Half-height above ground
3. Scale: (1, 1, 1) - 1m cube
4. Drag "RobotMetal.mat" onto cube
5. Press Play - cube should appear floating above ground
```

### Add Post-Processing (for photo-realism)
```
1. Window > Package Manager > Unity Registry
2. Search "Post Processing" > Install
3. GameObject > Volume > Global Volume
4. Inspector > Profile: New
5. Add Override > Bloom (glow effect)
6. Add Override > Ambient Occlusion (contact shadows)
7. Press Play - scene has enhanced lighting
```

### Add Grid Floor (for scale reference)
```
1. GameObject > 3D Object > Plane
2. Position: (0, 0, 0)
3. Scale: (10, 1, 10) - 100m × 100m
4. Material: Create new material with Grid texture
   - Albedo: Grid pattern (black lines on white)
   - Tiling: X: 10, Y: 10 (10cm grid squares)
```

---

## Saving the Scene

```
1. File > Save As
2. Navigate to Assets/Scenes/
3. File name: BasicScene.unity
4. Save
5. Verify scene appears in Project window under Assets/Scenes/
```

---

## Next Steps

Once BasicScene is working:
1. **Add Robot**: Import humanoid URDF or create GameObject hierarchy with Articulation Bodies
2. **Add Scripts**: Attach RobotController.cs to robot root GameObject
3. **Subscribe to /joint_states**: Test robot joint motion from ROS 2
4. **Progress to DigitalTwinScene**: Full scene with sensors, multiple robots, HRI assets

---

**Scene Version**: 1.0.0
**Tested On**: Unity 2021.3.16f1, Windows 11, Ubuntu 22.04 (via Windows Subsystem for Linux 2)
**Dependencies**: ROS-TCP-Connector v0.7.0+, ROS 2 Humble
