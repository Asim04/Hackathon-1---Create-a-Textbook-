# Chapter 2: Unity for High-Fidelity Simulation - Code Examples

**Module**: The Digital Twin (Gazebo & Unity)
**Chapter**: 2 - Unity for High-Fidelity Simulation
**Purpose**: Integrate Unity 2021.3 LTS with ROS 2 for photo-realistic robot visualization

---

## Files in this Directory

1. **`UnityProject_Structure.md`**: Complete Unity project layout, folder structure, package dependencies
2. **`BasicScene.md`**: Step-by-step guide to create minimal test scene (ground, lights, camera, ROS connection)
3. **`RobotController.cs`**: C# script to subscribe to `/joint_states`, update Articulation Bodies
4. **`ROSConnection.cs`**: C# script to initialize ROS-TCP-Connector, handle connection errors
5. **`README.md`**: This file - setup instructions, testing procedures, troubleshooting

---

## Prerequisites

### Software Requirements

**Operating System**:
- **Windows 10/11** (recommended for Unity development)
- **Ubuntu 22.04** (via WSL2 or dual boot) for ROS 2 Humble
- **macOS** (limited ROS 2 support, not recommended)

**Required Software**:
1. **Unity Hub** 3.x: Download from [unity.com/download](https://unity.com/download)
2. **Unity 2021.3 LTS**: Install via Unity Hub (Installs > Add > 2021.3.x LTS)
3. **ROS 2 Humble**: Already installed from Module 1
4. **Gazebo Classic 11**: Already installed from Chapter 1
5. **Python 3.10+**: For ROS-TCP-Endpoint

**Verification**:
```bash
# Check Unity installation (Windows PowerShell)
Get-ChildItem "C:\Program Files\Unity\Hub\Editor" | Select-Object Name
# Expected: 2021.3.xx folder

# Check ROS 2 (WSL2/Linux terminal)
ros2 --version
# Expected: ros2 doctor version 0.10.4

# Check Python
python3 --version
# Expected: Python 3.10.x or 3.11.x
```

---

## Installation Steps

### Step 1: Install Unity 2021.3 LTS

**If Unity Hub not installed**:
1. Download Unity Hub from https://unity.com/download
2. Run installer (Windows: UnityHubSetup.exe, macOS: UnityHub.dmg)
3. Accept license agreement
4. Complete installation (~200 MB download)

**Install Unity 2021.3 LTS**:
1. Open Unity Hub
2. Left sidebar > **Installs**
3. Click **Add** (top-right)
4. Select **2021.3.x LTS** (latest patch version)
5. Click **Next**
6. Add Modules (recommended):
   - ✓ Visual Studio Community 2019 (or 2022)
   - ✓ Windows Build Support (IL2CPP)
   - ✓ Linux Build Support (Mono) - if deploying to Linux
7. Click **Done** (~4-6 GB download, 10-15 minutes)
8. Verify installation: Unity Hub > Installs shows "2021.3.xx" with checkmark

---

### Step 2: Install ROS-TCP-Endpoint (Python Package)

**On Linux/WSL2 terminal**:
```bash
# Ensure ROS 2 Humble sourced
source /opt/ros/humble/setup.bash

# Install ROS-TCP-Endpoint via pip
pip3 install ros-tcp-endpoint

# Verify installation
python3 -c "import ros_tcp_endpoint; print('ROS-TCP-Endpoint installed successfully')"
# Expected: ROS-TCP-Endpoint installed successfully
```

**Test ROS-TCP-Endpoint**:
```bash
# Start endpoint server (should run without errors)
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# Expected output:
# [INFO] [ros_tcp_endpoint]: Starting ROS-TCP Endpoint on 0.0.0.0:10000
# [INFO] [ros_tcp_endpoint]: Listening for Unity connections...

# Press Ctrl+C to stop
```

---

### Step 3: Create Unity Project

1. **Open Unity Hub**
2. **Projects** tab > **New project** (top-right)
3. **Configure Project**:
   - Template: **3D (Core)** or **3D (URP)** - URP recommended for photo-realism
   - Project Name: `DigitalTwinVisualization`
   - Location: `D:\Projects\Unity\` (or preferred directory)
   - Unity Version: Select **2021.3.x LTS** from dropdown
4. Click **Create project** (takes 2-3 minutes to initialize)
5. Unity Editor opens with empty scene

---

### Step 4: Import ROS-TCP-Connector Package

**In Unity Editor**:
1. **Window > Package Manager** (opens Package Manager window)
2. Click **+ (Plus)** icon (top-left)
3. Select **Add package from git URL**
4. Enter URL:
   ```
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
   ```
5. Click **Add** (takes 30-60 seconds to download and import)
6. **Verification**: Package Manager shows "ROS TCP Connector" under **In Project**
   - Version: 0.7.0 or higher
   - Status: Installed (green checkmark)

**If import fails**:
- Check internet connection
- Verify Git is installed on Windows (`git --version` in PowerShell)
- Try manual installation: Clone repo, copy `com.unity.robotics.ros-tcp-connector` to `Packages/` folder

---

### Step 5: Create Project Structure

**In Unity Project window** (bottom):
1. Right-click **Assets** > **Create > Folder**
2. Create folders:
   - `Scripts`
   - `Scenes`
   - `Prefabs`
   - `Materials`
   - `Models`
3. Final structure:
   ```
   Assets/
   ├── Materials/
   ├── Models/
   ├── Prefabs/
   ├── Scenes/
   └── Scripts/
   ```

---

### Step 6: Add C# Scripts

**Copy Scripts to Unity**:
1. Copy `RobotController.cs` from this directory
2. In Unity Editor > Project window > `Assets/Scripts/`
3. **Right-click > Show in Explorer** (opens folder in Windows Explorer)
4. Paste `RobotController.cs` and `ROSConnection.cs` into this folder
5. Return to Unity Editor - scripts appear in Scripts folder (Unity auto-detects)
6. **Verify**: Double-click scripts - should open in Visual Studio/Rider without compile errors

**If scripts show errors**:
- Check ROS-TCP-Connector is installed (Package Manager)
- Restart Unity Editor (File > Close Project, then reopen)
- Check script file encoding is UTF-8 (not UTF-16)

---

### Step 7: Create Basic Scene

Follow instructions in `BasicScene.md` to create minimal test scene:
1. Create ground plane (Quad, 10m × 10m)
2. Add directional light (Sun, intensity 1.0)
3. Position camera (5, 3, -5) looking at origin
4. Add ROSConnection GameObject
5. Save scene as `Assets/Scenes/BasicScene.unity`

**Quick Test**:
```
1. Press Play button (top center of Unity Editor)
2. Game view shows ground plane + light
3. HUD (top-left) displays "Status: Connecting..." (yellow)
4. Press Play again to stop
```

---

## Testing and Validation

### Test 1: ROS Connection (No Robot)

**Terminal 1 - Start ROS-TCP-Endpoint**:
```bash
source /opt/ros/humble/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**Unity Editor**:
1. Open `BasicScene.unity`
2. Select **ROSConnection** GameObject in Hierarchy
3. Inspector > **ROS Connection** component:
   - ROS IP Address: `127.0.0.1` (localhost)
   - ROS Port: `10000`
   - Protocol: ROS 2
   - Show HUD: ✓
4. Press **Play**
5. **Expected Result**: HUD changes to "Status: Connected" (green) within 2-3 seconds

**Success Criteria**:
- [x] Unity connects to ROS-TCP-Endpoint (green HUD)
- [x] Console shows: "ROSConnection: Successfully connected to 127.0.0.1:10000"
- [x] No error messages in Console
- [x] Frame rate ≥30 FPS (Profiler window)

---

### Test 2: Joint State Subscription (With Robot)

**Prerequisites**:
- Test 1 passed (connection established)
- Robot GameObject with Articulation Bodies in scene

**Terminal 1 - ROS-TCP-Endpoint**:
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**Terminal 2 - Publish Test Joint States**:
```bash
source /opt/ros/humble/setup.bash

# Publish single joint state message
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'},
  name: ['right_shoulder_pitch', 'right_elbow_roll'],
  position: [0.5, 1.0],
  velocity: [],
  effort: []
}" --once
```

**Unity Editor**:
1. Select robot root GameObject
2. Inspector > Add Component > **RobotController**
3. Drag robot root's **Articulation Body** into **Robot Root** field
4. Check **Debug Mode** for detailed logs
5. Press **Play**
6. **Expected Result**: Robot joints move to positions [0.5, 1.0] radians (~28.6°, 57.3°)

**Success Criteria**:
- [x] Console shows: "RobotController: Subscribed to /joint_states"
- [x] Console shows: "RobotController: Updated 'right_shoulder_pitch' to 28.65° (0.500 rad)"
- [x] Robot joints visibly rotate in Scene/Game view
- [x] No latency warnings (<50ms)

---

### Test 3: Full Integration (Gazebo + Unity)

**Terminal 1 - ROS-TCP-Endpoint**:
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**Terminal 2 - Launch Gazebo with Robot** (from Chapter 1):
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws  # Your workspace from Module 1
source install/setup.bash

# Launch Gazebo with humanoid robot
ros2 launch module_2_gazebo_examples launch_gazebo.launch.py
```

**Unity Editor**:
1. Open `DigitalTwinScene.unity` (with full robot setup)
2. RobotController attached to robot, configured for `/joint_states`
3. Press **Play**
4. **Expected Result**:
   - Unity robot mirrors Gazebo robot motion in real-time
   - Joint angles synchronized within 50ms
   - Frame rate ≥30 FPS

**Success Criteria**:
- [x] Unity HUD shows "Connected" (green)
- [x] Robot in Unity moves identically to Gazebo robot
- [x] Console shows joint updates at ~1000 Hz (check with Debug Mode)
- [x] Latency <50ms (Console logs if Debug Mode enabled)

---

## Troubleshooting

### Issue 1: "Package ROS-TCP-Connector not found"

**Cause**: Git URL incorrect, network issue, or Git not installed

**Solution**:
```powershell
# Check Git installation (Windows PowerShell)
git --version
# If error, install Git: https://git-scm.com/download/win

# Verify internet connection to GitHub
Test-Connection github.com -Count 4

# Retry package import with exact URL (copy-paste):
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
```

---

### Issue 2: "Status: Connecting..." (yellow, never turns green)

**Cause**: ROS-TCP-Endpoint not running, firewall blocking port 10000, or incorrect IP

**Solution**:
```bash
# Verify ROS-TCP-Endpoint is running
ps aux | grep ros_tcp_endpoint
# Should show: python3 ... ros_tcp_endpoint

# Check port 10000 is listening
netstat -an | grep 10000
# Expected: tcp 0.0.0.0:10000 ... LISTEN

# Test connection manually (from Unity machine)
telnet 127.0.0.1 10000
# Should connect without error

# If firewall issue (Windows):
# Control Panel > Windows Defender Firewall > Advanced > Inbound Rules
# Add rule: Allow TCP port 10000 for Python
```

---

### Issue 3: Joints don't move in Unity

**Cause**: Joint name mismatch between ROS and Unity, or xDrive not configured

**Diagnosis**:
```csharp
// Enable Debug Mode in RobotController Inspector
// Console will show:
// "RobotController: Joint 'right_shoulder_pitch' from ROS not found in Unity hierarchy"
```

**Solution**:
1. **Check Unity GameObject names**:
   - Select robot in Hierarchy
   - Expand all children
   - Compare names to ROS joint names (from `ros2 topic echo /joint_states`)

2. **Rename Unity GameObjects** to match ROS exactly:
   - ROS: `right_shoulder_pitch` → Unity: Rename GameObject to `right_shoulder_pitch`
   - Case-sensitive! `Right_Shoulder_Pitch` ≠ `right_shoulder_pitch`

3. **Verify Articulation Body xDrive**:
   - Select joint GameObject
   - Inspector > Articulation Body > xDrive
   - Stiffness: 10000 (if too low, joint won't move)
   - Damping: 100 (if too high, joint moves slowly)
   - Force Limit: 1000 (if too low, joint underpowered)

---

### Issue 4: Jittery or oscillating joints

**Cause**: xDrive stiffness too high, creating spring-like behavior

**Solution**:
```csharp
// Select joint GameObject in Unity
// Inspector > Articulation Body > xDrive

// Reduce stiffness for smoother motion
Stiffness: 5000 (down from 10000)
Damping: 500 (up from 100)

// Alternatively, increase target velocity (if using velocity control)
xDrive.targetVelocity = 2.0f; // rad/s
```

---

### Issue 5: High latency (>50ms warnings in Console)

**Cause**: Network delay, Unity frame rate too low, or ROS-TCP-Endpoint overloaded

**Solution**:
1. **Run ROS-TCP-Endpoint on same machine as Unity**:
   - Windows: Install WSL2, run ROS 2 Humble in WSL
   - IP Address: `127.0.0.1` (localhost)

2. **Increase Unity target frame rate**:
   ```csharp
   // Add to ROSConnection.cs Start() method
   Application.targetFrameRate = 60; // Force 60 FPS
   ```

3. **Enable VSync**:
   ```
   Edit > Project Settings > Quality
   VSync Count: Every V Blank (locks to monitor refresh rate, usually 60 Hz)
   ```

4. **Check ROS message rate**:
   ```bash
   ros2 topic hz /joint_states
   # If < 100 Hz, increase Gazebo update rate or joint_state_publisher frequency
   ```

---

### Issue 6: Robot mesh appears pink/missing textures

**Cause**: Materials not assigned or textures missing from imported model

**Solution**:
1. **Check Materials folder**:
   - Project window > Assets/Materials
   - Should contain .mat files (RobotMetal, RobotPlastic, etc.)

2. **Assign materials to robot mesh**:
   - Select robot mesh GameObject
   - Inspector > Mesh Renderer > Materials
   - Drag appropriate .mat file from Materials folder

3. **Re-import robot model** (if using URDF):
   - Assets > Import Robot from URDF
   - Check "Import Materials" option
   - Select correct mesh format (FBX, OBJ, or STL)

---

### Issue 7: Unity Editor crashes on Play

**Cause**: Infinite loop in script, or Unity version incompatibility

**Solution**:
1. **Check Unity version**:
   ```
   Help > About Unity
   Version should be: 2021.3.x (NOT 2022.x or 2020.x)
   ```

2. **Check for infinite loops in custom scripts**:
   ```csharp
   // BAD: Infinite loop blocks Unity
   while (true) {
       UpdateJoints();
   }

   // GOOD: Use Update() or coroutine
   void Update() {
       UpdateJoints();
   }
   ```

3. **Review Console for errors before crash**:
   - Window > General > Console
   - Look for red error messages preceding crash
   - Common: NullReferenceException (missing GameObject assignment)

---

## Network Configuration

### Local Setup (Unity + ROS on Same Machine)

**Unity Settings**:
- ROS IP Address: `127.0.0.1`
- ROS Port: `10000`

**ROS-TCP-Endpoint Command**:
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
```

---

### Remote Setup (Unity on Windows, ROS on Linux)

**Scenario**: Unity on Windows PC (192.168.1.100), ROS 2 on Ubuntu laptop (192.168.1.200)

**Unity Settings**:
- ROS IP Address: `192.168.1.200` (Linux machine's LAN IP)
- ROS Port: `10000`

**Linux Firewall** (Ubuntu):
```bash
# Allow TCP port 10000
sudo ufw allow 10000/tcp
sudo ufw reload

# Verify firewall status
sudo ufw status
# Should show: 10000/tcp ALLOW Anywhere
```

**ROS-TCP-Endpoint Command** (on Linux):
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
```

**Test connectivity** (from Windows):
```powershell
Test-NetConnection -ComputerName 192.168.1.200 -Port 10000
# Expected: TcpTestSucceeded : True
```

---

### WSL2 Setup (Unity on Windows, ROS on WSL2)

**Find WSL2 IP Address**:
```bash
# In WSL2 terminal
hostname -I
# Output example: 172.24.123.45 (use this IP in Unity)
```

**Unity Settings**:
- ROS IP Address: `172.24.123.45` (WSL2 IP from above)
- ROS Port: `10000`

**ROS-TCP-Endpoint Command** (in WSL2):
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**Note**: WSL2 IP changes on reboot. Use this PowerShell script to auto-get IP:
```powershell
wsl hostname -I
```

---

## Performance Optimization

### Target Frame Rate: 60 FPS

**Why**: 60 FPS = 16.67ms frame time, leaves 33ms budget for network latency (<50ms total per SC-002)

**How to Enforce**:
```csharp
// Add to ROSConnection.cs Start() method
void Start()
{
    Application.targetFrameRate = 60;
    QualitySettings.vSyncCount = 1; // Enable VSync
}
```

---

### Reduce Visual Quality for Performance

**For low-end hardware** (< GTX 1060 / RX 580):
```
Edit > Project Settings > Quality
Select: Low or Medium

Changes:
- Shadow Resolution: Low Resolution
- Anti Aliasing: Disabled or 2x MSAA
- Pixel Light Count: 1
- Real-time Reflection Probes: Off
```

**Expected Improvement**: +15-20 FPS, with minor visual quality loss

---

### Simplify Robot Mesh

**For high polygon count models** (>50k triangles):
1. Use Blender or MeshLab to decimate mesh
2. Target: 10k-20k triangles for real-time rendering
3. Keep collision meshes simple (use primitive shapes: box, capsule, sphere)

**Unity LOD (Level of Detail)**:
```
Select robot mesh > Inspector > Add Component > LOD Group
LOD 0 (100%): Full detail mesh
LOD 1 (50%): Medium detail (50% triangles)
LOD 2 (25%): Low detail (25% triangles)
```

---

## Next Steps

After completing Chapter 2 setup:
1. **Add Robot Sensors** (Chapter 3):
   - Subscribe to `/scan` (LiDAR point cloud)
   - Subscribe to `/camera/depth/image_raw` (depth camera)
   - Visualize sensor data in Unity scene

2. **Implement Control** (Chapter 4):
   - Publish `/cmd_vel` from Unity (keyboard/joystick input)
   - Close the loop: Unity → ROS 2 → Gazebo → ROS 2 → Unity

3. **Optimize Performance**:
   - Profile with Unity Profiler (Window > Analysis > Profiler)
   - Reduce draw calls with mesh batching
   - Use object pooling for dynamic objects (sensor rays, particles)

---

**README Version**: 1.0.0
**Last Updated**: 2025-12-18
**Tested On**: Windows 11, Unity 2021.3.16f1, ROS 2 Humble, Ubuntu 22.04 (WSL2)
**Estimated Setup Time**: 1-2 hours (first time), 15-30 minutes (subsequent)
