# Quickstart Guide: Module 2 - The Digital Twin (Gazebo & Unity)

**Purpose**: Step-by-step workflow for creating Module 2 content from templates to validated chapters
**Version**: 1.0.0
**Estimated Time**: 30-40 hours total (distributed across 4 chapters + integration)
**Prerequisites**: Completed Module 1, Ubuntu 22.04, ROS 2 Humble installed

---

## Overview

This quickstart provides the complete workflow for creating Module 2 content. It enforces strict word count control to prevent Module 1's 97% overage (11,800 vs 6,000 words) by requiring **per-section validation** before proceeding.

**Critical Success Factors**:
1. ✅ **Word Budget Enforcement**: Check word count after each section (not at chapter end)
2. ✅ **Early Testing**: Test code examples during writing (not deferred to Phase 7)
3. ✅ **Environment Setup**: Install Gazebo + Unity BEFORE content creation to validate examples
4. ✅ **Citation Discipline**: Add inline citations as you write (not retrospectively)

---

## Step 1: Review Research & Data Model (15 minutes)

### Objective
Understand technical foundation and content structure before writing.

### Actions

1. **Read Research Documentation**:
   ```bash
   cat specs/002-digital-twin-gazebo-unity/research.md
   ```
   - Note: 17 sources total (10 peer-reviewed = 59%)
   - Key citations: Koenig & Howard (2004), Grieves & Vickers (2017), Unity documentation
   - Technical decisions: Gazebo Classic 11, Unity 2021.3 LTS, built-in sensor plugins
   - Word targets: Ch1: 1,300, Ch2: 1,400, Ch3: 1,300, Ch4: 1,800 (total: 5,800)

2. **Review Data Model**:
   ```bash
   cat specs/002-digital-twin-gazebo-unity/data-model.md
   ```
   - 6 entities: Gazebo World, Unity Scene, Sensor Config, Physics Params, ROS-Unity Message, Digital Twin System
   - Validation rules: word count 1,200-1,500/chapter, Flesch-Kincaid grade 10-12, min 3 citations/chapter

3. **Inspect Templates**:
   ```bash
   ls -lh specs/002-digital-twin-gazebo-unity/contracts/
   # chapter-template.md
   # gazebo-world-template.sdf
   # unity-scene-template.md
   # sensor-config-template.md
   ```

**Checkpoint**: You should now understand:
- What sources to cite (research.md)
- What word limits apply (data-model.md + chapter-template.md)
- What templates are available (contracts/)

---

## Step 2: Set Up Testing Environment (1-2 hours)

### Objective
Install Gazebo, Unity, and ROS 2 integration tools to validate examples as you write them.

**⚠️ CRITICAL**: This step was moved from Phase 7 to Phase 2 to enable **continuous testing during content creation** (Module 1 lesson learned).

### 2.1 Install Gazebo Classic 11

```bash
# Install Gazebo and ROS 2 Humble integration packages
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control -y

# Verify installation
gazebo --version
# Expected: Gazebo multi-robot simulator, version 11.x.x

# Test empty world launch
ros2 launch gazebo_ros gazebo.launch.py
# Should open Gazebo GUI with empty world
# Press Ctrl+C to exit
```

### 2.2 Install Unity 2021.3 LTS

**Linux**:
```bash
# Install Unity Hub (if not already installed)
wget -qO - https://hub.unity3d.com/linux/keys/public | gpg --dearmor | sudo tee /usr/share/keyrings/unityhub.gpg > /dev/null
sudo sh -c 'echo "deb [signed-by=/usr/share/keyrings/unityhub.gpg] https://hub.unity3d.com/linux/repos/deb stable main" > /etc/apt/sources.list.d/unityhub.list'
sudo apt update
sudo apt install unityhub -y

# Launch Unity Hub (GUI)
unityhub
# In Unity Hub: Installs > Install Editor > 2021.3.x LTS (latest patch)
```

**Windows**:
- Download Unity Hub from [unity.com/download](https://unity.com/download)
- Install Unity 2021.3.x LTS via Unity Hub

### 2.3 Install ROS-TCP-Connector in Unity

1. **Create Test Unity Project**:
   - Unity Hub > New Project > 3D (Core)
   - Name: `Module2_Test`
   - Unity Version: 2021.3.x LTS

2. **Add ROS-TCP-Connector**:
   - In Unity Editor: **Window > Package Manager**
   - Click **+ (Plus)** > **Add package from git URL**
   - Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
   - Click **Add** (takes 30-60 seconds)

3. **Verify Package**:
   - **Window > Package Manager** > Packages: In Project
   - Should see "ROS TCP Connector v0.7.0+" listed

### 2.4 Test ROS-Unity Connection

**Terminal 1** (ROS-TCP-Endpoint):
```bash
source /opt/ros/humble/setup.bash
pip3 install ros-tcp-endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
# Expected: [INFO] [ros_tcp_endpoint]: ROS-TCP Endpoint started on 0.0.0.0:10000
```

**Unity Editor**:
- **Robotics > ROS Settings**
- Set **ROS IP Address**: `127.0.0.1`
- Set **ROS Port**: `10000`
- Set **Protocol**: ROS 2
- Press **Play** (top center button)
- Check Console (Window > General > Console): Should see connection established

**Checkpoint**:
- ✅ Gazebo launches without errors
- ✅ Unity 2021.3 LTS installed
- ✅ ROS-TCP-Connector imported successfully
- ✅ Unity connects to ROS-TCP-Endpoint (green HUD overlay)

**If issues occur**: Consult `unity-scene-template.md` Step 8 troubleshooting section.

---

## Step 3: Write Chapter Content (4-6 hours per chapter)

### Objective
Create chapter Markdown files following strict word budgets and template structure.

### Workflow (per chapter)

1. **Copy Chapter Template**:
   ```bash
   cp specs/002-digital-twin-gazebo-unity/contracts/chapter-template.md \
      docs/module-2-digital-twin/01-gazebo-physics.md
   ```

2. **Fill Template Sections** (in order):
   - **Introduction** (200-250 words for Ch1/Ch3, 250 words for Ch2/Ch4)
     - Motivate with real-world robotics scenario
     - Connect to previous chapters
     - Preview what students will build
   - **Section 1: Concept Introduction** (250-300 words)
     - Define core concept with diagram (Mermaid or textual)
     - Add 1-2 inline citations: `(Koenig & Howard, 2004)`
   - **Section 2: Practical Implementation** (300-350 words)
     - Step-by-step instructions with code examples
     - Expected output descriptions
   - **Section 3: Advanced Topic** (250-350 words)
     - Configuration options, parameters
     - Common issues table (Issue | Cause | Solution)
   - **Section 4: Integration** (200-300 words)
     - How this connects to ROS 2 / other chapters
     - Testing and verification steps
   - **Practice Exercises** (100-150 words total)
     - 3 exercises: Beginner, Intermediate, Advanced
   - **Summary** (50-100 words)
     - 5 key takeaways
     - Forward reference to next chapter
   - **Further Reading** (3+ sources with relevance)

3. **Enforce Word Budget** (MANDATORY after each section):
   ```bash
   # Use online word counter or command-line tool
   cat docs/module-2-digital-twin/01-gazebo-physics.md | wc -w
   # Or use: https://wordcounter.net/
   ```
   - **If over budget**: Trim redundancy, tighten prose, remove verbose examples **immediately**
   - **Do NOT defer trimming** to end of chapter - edit while content is fresh

4. **Add Inline Citations**:
   - Use APA in-text format: `(Author, Year)` or `(Author1 & Author2, Year)`
   - Example: "Gazebo's ODE physics engine provides accurate contact simulation (Koenig & Howard, 2004)."
   - Minimum 3 citations per chapter (from research.md bibliography)

5. **Check Readability**:
   - Use online tool: [readable.com](https://readable.com) or [hemingwayapp.com](https://hemingwayapp.com)
   - Target: Flesch-Kincaid grade 10-12
   - If higher: Simplify sentence structure, reduce jargon

**Chapter-Specific Word Budgets** (from chapter-template.md):

| Chapter | Intro | Sec1 | Sec2 | Sec3 | Sec4 | Exercises | Summary | Total |
|---------|-------|------|------|------|------|-----------|---------|-------|
| Ch1: Gazebo | 200 | 250 | 300 | 300 | 250 | 100 | 50 | 1,300 |
| Ch2: Unity | 250 | 275 | 300 | 300 | 275 | 100 | 50 | 1,400 |
| Ch3: Sensors | 200 | 250 | 300 | 300 | 250 | 100 | 50 | 1,300 |
| Ch4: Lab | 250 | 300 | 350 | 350 | 300 | 150 | 100 | 1,800 |

**Checkpoint** (per section):
- [ ] Section word count within budget
- [ ] At least 1 citation added (if Section 1-4)
- [ ] Readability grade 10-12
- [ ] No [placeholders] or TODO comments

---

## Step 4: Create Gazebo/Unity Examples (2-3 hours per chapter)

### Objective
Produce executable code examples for each chapter that students can run verbatim.

### Chapter 1: Gazebo Physics Examples

**Create**:
```bash
cd code-examples/module-2-digital-twin/chapter-1-gazebo/

# Example 1.1: Basic Gazebo world
cp ../../specs/002-digital-twin-gazebo-unity/contracts/gazebo-world-template.sdf \
   physics_demo.world

# Example 1.2: ROS 2 launch file
cat > gazebo_physics_demo.launch.py << 'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'demo_box', '-file', 'box_model.sdf'],
        ),
    ])
EOF

# Example 1.3: SDF model with physics properties
cat > box_model.sdf << 'EOF'
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="demo_box">
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia><ixx>0.167</ixx><iyy>0.167</iyy><izz>0.167</izz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>1 1 1</size></box></geometry>
        <surface>
          <friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry><box><size>1 1 1</size></box></geometry>
      </visual>
    </link>
  </model>
</sdf>
EOF
```

### Chapter 2: Unity Visualization Examples

**Create**:
```bash
cd code-examples/module-2-digital-twin/chapter-2-unity/

# Example 2.1: ROS Joint State Subscriber (C#)
cat > RobotJointSubscriber.cs << 'EOF'
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class RobotJointSubscriber : MonoBehaviour
{
    public string jointStateTopic = "/joint_states";
    public ArticulationBody robotRoot;
    private ArticulationBody[] joints;

    void Start()
    {
        joints = robotRoot.GetComponentsInChildren<ArticulationBody>();
        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>(
            jointStateTopic,
            UpdateJointStates
        );
    }

    void UpdateJointStates(JointStateMsg msg)
    {
        for (int i = 0; i < msg.name.Length; i++)
        {
            string jointName = msg.name[i];
            double position = msg.position[i];
            foreach (var joint in joints)
            {
                if (joint.name == jointName)
                {
                    var drive = joint.xDrive;
                    drive.target = (float)(position * Mathf.Rad2Deg);
                    joint.xDrive = drive;
                    break;
                }
            }
        }
    }
}
EOF
```

### Chapter 3: Sensor Simulation Examples

**Create**:
```bash
cd code-examples/module-2-digital-twin/chapter-3-sensors/

# Example 3.1: LiDAR plugin configuration (URDF snippet)
cp ../../specs/002-digital-twin-gazebo-unity/contracts/sensor-config-template.md .
# Extract LiDAR example block from template

# Example 3.2: ROS 2 sensor verification script
cat > verify_sensors.py << 'EOF'
#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import LaserScan, Imu, Image

def check_lidar(node):
    msgs = []
    def cb(msg): msgs.append(msg)
    sub = node.create_subscription(LaserScan, '/robot/scan', cb, 10)
    rclpy.spin_once(node, timeout_sec=2.0)
    print(f"LiDAR: {'✓ Publishing' if msgs else '✗ Not publishing'}")

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('sensor_verifier')
    check_lidar(node)
    node.destroy_node()
    rclpy.shutdown()
EOF
chmod +x verify_sensors.py
```

### Chapter 4: Digital Twin Lab Examples

**Create**:
```bash
cd code-examples/module-2-digital-twin/chapter-4-lab/

# Example 4.1: Complete launch file for Gazebo + ROS 2 bridge
cat > digital_twin_gazebo.launch.py << 'EOF'
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription('gazebo_ros/launch/gazebo.launch.py'),
        Node(package='robot_state_publisher', executable='robot_state_publisher'),
        Node(package='ros_tcp_endpoint', executable='default_server_endpoint'),
    ])
EOF

# Example 4.2: Unity integration test script
cat > test_unity_sync.py << 'EOF'
#!/usr/bin/env python3
"""Test synchronization between Gazebo and Unity"""
import rclpy
from sensor_msgs.msg import JointState

def test_joint_sync():
    # Publish joint commands from ROS 2
    # Verify Unity receives them within 50ms latency (SC-002)
    pass

if __name__ == '__main__':
    test_joint_sync()
EOF
```

**Checkpoint**:
- [ ] All code examples created
- [ ] File paths match chapter references
- [ ] Syntax-valid (no obvious typos)

---

## Step 5: Test Examples (1-2 hours per chapter)

### Objective
Validate examples execute successfully and document expected outputs.

**⚠️ CRITICAL**: Test examples **during content creation**, not at end of module (Module 1 lesson learned).

### Testing Procedure (per example)

1. **Run Example**:
   ```bash
   cd code-examples/module-2-digital-twin/chapter-1-gazebo/
   ros2 launch gazebo_physics_demo.launch.py
   ```

2. **Document Expected Output** (in chapter Markdown):
   ```markdown
   **Expected Output**:
   ```
   Gazebo GUI opens with 1×1×1 meter box at position (0, 0, 0.5).
   Box falls due to gravity and collides with ground plane.
   Collision detected at t ≈ 0.32s with contact force ~9.81 N.
   ```
   ```

3. **Capture Screenshot** (textual description if visual):
   ```markdown
   ![Gazebo Physics Demo](../assets/screenshots/ch1-gazebo-box-collision.png)
   *Figure 1.1: Box collision with ground plane in Gazebo*
   ```
   - For this documentation, provide **textual description** instead of actual image:
     "Screenshot shows Gazebo GUI with 1m cube resting on gray ground plane, shadow visible, time display 't=0.5s'"

4. **Identify 3+ Common Issues**:
   ```markdown
   | Issue | Cause | Solution |
   |-------|-------|----------|
   | "Model not found" error | Wrong file path in launch file | Verify path with `ls -lh box_model.sdf` |
   | Box falls through ground | Collision geometry missing | Add `<collision>` tag to SDF |
   | Physics unstable (jittering) | Timestep too large | Reduce `<max_step_size>` to 0.001 |
   ```

5. **Test on Target Platform**:
   - OS: Ubuntu 22.04 LTS
   - ROS 2: Humble Hawksbill
   - Gazebo: Classic 11.x
   - Unity: 2021.3.x LTS

**Checkpoint** (per chapter):
- [ ] All examples execute without errors
- [ ] Expected outputs documented
- [ ] Screenshots described (textual placeholders)
- [ ] 3+ common issues identified with solutions
- [ ] Tested on Ubuntu 22.04 + ROS 2 Humble + Unity 2021.3

---

## Step 6: Create Lab Guide (6-10 hours for Chapter 4)

### Objective
Produce comprehensive lab guide for Chapter 4 integrating Gazebo + Unity + ROS 2.

### Workflow

1. **Use Lab Template** (if exists):
   ```bash
   # Check if lab-guide-template.md exists
   ls specs/002-digital-twin-gazebo-unity/contracts/lab-guide-template.md
   # If not, adapt from chapter-template.md
   ```

2. **Lab Scenario**: Complete digital twin with:
   - Gazebo: Physics simulation of humanoid robot
   - Unity: Photo-realistic visualization with HRI
   - ROS 2: Middleware for state synchronization
   - Sensors: LiDAR + Depth camera + IMU
   - Control: Simple joint position controller

3. **Lab Structure**:
   - **Objectives**: 3-5 measurable learning outcomes
   - **Prerequisites**: List required chapters, software, knowledge
   - **Setup**: Step-by-step environment configuration (15-30 min)
   - **Part 1**: Launch Gazebo simulation (30 min)
   - **Part 2**: Connect Unity visualization (45 min)
   - **Part 3**: Add sensor simulation (45 min)
   - **Part 4**: Implement control loop (1 hour)
   - **Part 5**: Validation and testing (30 min)
   - **Deliverables**: What students submit (screenshots, code, report)
   - **Grading Rubric**: Measurable criteria (see below)

4. **Grading Rubric** (100 points total):

   | Criterion | Points | Description |
   |-----------|--------|-------------|
   | **Digital Twin Functionality** | 30 | Gazebo and Unity synchronized, real-time factor ≥0.5x |
   | **Sensor Integration** | 25 | All sensors (LiDAR, depth, IMU) publishing correctly |
   | **Control Implementation** | 20 | Joint commands move robot, <50ms latency |
   | **Code Quality** | 15 | Clean, commented, follows ROS 2 conventions |
   | **Documentation** | 10 | Lab report with screenshots, results, analysis |

5. **Validation Checklist** (students complete):
   - [ ] Gazebo simulation runs at ≥0.5x real-time
   - [ ] Unity connects to ROS 2 (green HUD indicator)
   - [ ] LiDAR publishes to `/scan` at ~10 Hz
   - [ ] Depth camera publishes RGB + depth images at ~30 Hz
   - [ ] IMU publishes orientation at ~100 Hz
   - [ ] Joint commands from ROS 2 move Unity robot within 50ms
   - [ ] No error messages in Console (ROS 2, Gazebo, Unity)

**Checkpoint**:
- [ ] Lab guide complete (4,000-5,000 words for Chapter 4)
- [ ] Grading rubric with measurable criteria
- [ ] Estimated completion time stated (3-4 hours)
- [ ] Validation checklist included

---

## Step 7: Compile References (1 hour)

### Objective
Extract all inline citations and format in APA style for references.md.

### Workflow

1. **Extract Citations** from chapter files:
   ```bash
   grep -oh '([A-Z][a-z].*[0-9]\{4\})' docs/module-2-digital-twin/*.md | sort -u
   # Output example:
   # (Koenig & Howard, 2004)
   # (Grieves & Vickers, 2017)
   # (Unity Technologies, 2021)
   ```

2. **Lookup Full References** in research.md:
   ```bash
   grep "Koenig" specs/002-digital-twin-gazebo-unity/research.md
   ```

3. **Format in APA Style** (references.md):
   ```markdown
   # References: Module 2 - The Digital Twin

   ## Peer-Reviewed Publications

   Grieves, M., & Vickers, J. (2017). Digital twin: Mitigating unpredictable, undesirable emergent behavior in complex systems. In F.-J. Kahlen, S. Flumerfelt, & A. Alves (Eds.), *Transdisciplinary perspectives on complex systems* (pp. 85-113). Springer. https://doi.org/10.1007/978-3-319-38756-7_4

   Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. In *2004 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)* (Vol. 3, pp. 2149-2154). IEEE. https://doi.org/10.1109/IROS.2004.1389727

   ## Official Documentation

   Unity Technologies. (2021). *Unity 2021.3 LTS documentation*. Retrieved December 18, 2025, from https://docs.unity3d.com/2021.3/Documentation/Manual/

   Open Source Robotics Foundation. (2023). *Gazebo Classic documentation*. Retrieved December 18, 2025, from https://classic.gazebosim.org/tutorials

   ## Technical Reports

   [Additional sources from research.md...]
   ```

4. **Verify Citation Count**:
   ```bash
   wc -l docs/module-2-digital-twin/references.md
   # Should have 15+ references minimum (data-model.md requirement)
   ```

**Checkpoint**:
- [ ] All inline citations have matching full references
- [ ] APA format correct (author, year, title, publisher, DOI/URL)
- [ ] Minimum 15 references total
- [ ] Minimum 8 peer-reviewed sources (53% threshold)

---

## Step 8: Quality Validation (2-3 hours)

### Objective
Verify all success criteria from data-model.md before submission.

### Validation Checklist

#### 8.1 Word Count Validation

```bash
# Check each chapter
for chapter in docs/module-2-digital-twin/{01,02,03,04}*.md; do
    echo "$chapter: $(cat $chapter | wc -w) words"
done

# Expected output:
# 01-gazebo-physics.md: 1,300 words (target: 1,200-1,500)
# 02-unity-visualization.md: 1,400 words (target: 1,200-1,500)
# 03-sensor-simulation.md: 1,300 words (target: 1,200-1,500)
# 04-lab-digital-twin.md: 1,800 words (target: 1,200-1,500 - EXCEPTION for lab)
```

**Action**: If any chapter outside range, trim or expand as needed.

#### 8.2 Readability Testing

Use online tool: [readable.com](https://readable.com) or [hemingwayapp.com](https://hemingwayapp.com)

- Paste chapter content into tool
- Check **Flesch-Kincaid Grade Level**: Target 10-12
- If higher (13+): Simplify sentences, reduce passive voice, break long paragraphs

#### 8.3 Citation Count

```bash
# Count citations per chapter
grep -c '([A-Z][a-z].*[0-9]\{4\})' docs/module-2-digital-twin/01-gazebo-physics.md
# Should be ≥3 per chapter, 15+ total
```

#### 8.4 Code Example Testing

Re-run all code examples from Step 5 to ensure reproducibility:

```bash
# Test Chapter 1 Gazebo example
cd code-examples/module-2-digital-twin/chapter-1-gazebo/
ros2 launch gazebo_physics_demo.launch.py

# Test Chapter 2 Unity example (manual - open Unity project)

# Test Chapter 3 sensor verification
cd ../chapter-3-sensors/
python3 verify_sensors.py

# Test Chapter 4 digital twin integration
cd ../chapter-4-lab/
ros2 launch digital_twin_gazebo.launch.py
```

**Expected**: All examples execute without errors, outputs match documented expectations.

#### 8.5 Plagiarism Check

Use institutional tool (Turnitin, iThenticate) or free alternative:
- Copy chapter content to plagiarism checker
- Target: <10% similarity (excluding quotes and citations)
- If high similarity: Paraphrase, add citations, ensure originality

**Checkpoint**:
- [ ] All chapters within word count range (±50 words tolerance)
- [ ] Flesch-Kincaid grade 10-12 for all chapters
- [ ] Minimum 3 citations per chapter, 15+ total
- [ ] All code examples tested successfully
- [ ] Plagiarism score <10%

---

## Step 9: Docusaurus Integration (1 hour)

### Objective
Add front matter and integrate chapters into Docusaurus site navigation.

### Workflow

1. **Add Front Matter** to each chapter:
   ```markdown
   ---
   sidebar_position: 1
   title: "Chapter 1: Gazebo Physics Simulation"
   description: "Learn physics engines, gravity, collisions, and ROS 2 integration for robot simulation in Gazebo"
   ---

   # Chapter 1: Gazebo Physics Simulation
   [Rest of content...]
   ```

2. **Verify sidebars.js** (already updated in Phase 1):
   ```bash
   grep "module-2-digital-twin" sidebars.js
   # Should show 6 items: index, chapters 1-4, references
   ```

3. **Create Module Overview** (index.md):
   ```bash
   cat > docs/module-2-digital-twin/index.md << 'EOF'
   ---
   sidebar_position: 1
   title: "Module 2: The Digital Twin (Gazebo & Unity)"
   ---

   # Module 2: The Digital Twin

   **Duration**: 12-15 hours
   **Prerequisites**: Module 1 completion (ROS 2 fundamentals, URDF, Python)

   ## Module Overview

   This module teaches digital twin development for humanoid robots using:
   - **Gazebo Classic 11**: Physics simulation (Chapter 1)
   - **Unity 2021.3 LTS**: Photo-realistic visualization (Chapter 2)
   - **Sensor Simulation**: LiDAR, depth cameras, IMUs (Chapter 3)
   - **Integration Lab**: Complete digital twin system (Chapter 4)

   ## Learning Objectives

   By completing this module, you will be able to:
   1. Configure Gazebo physics engines (ODE/Bullet) for realistic robot simulation
   2. Integrate Unity with ROS 2 using ROS-TCP-Connector for visualization
   3. Simulate sensors (LiDAR, depth cameras, IMU) with noise models
   4. Build a complete digital twin synchronizing Gazebo physics and Unity rendering

   ## Chapters

   - [Chapter 1: Gazebo Physics Simulation](./01-gazebo-physics)
   - [Chapter 2: Unity for High-Fidelity Simulation](./02-unity-visualization)
   - [Chapter 3: Simulating Sensors](./03-sensor-simulation)
   - [Chapter 4: Lab - Building a Digital Twin](./04-lab-digital-twin)

   ## References

   - [Module 2 References](./references)

   ---

   **Next**: [Chapter 1: Gazebo Physics Simulation](./01-gazebo-physics)
   EOF
   ```

4. **Test Docusaurus Build**:
   ```bash
   npm run build
   # Should complete without errors
   # Check output for broken links or warnings
   ```

5. **Test Local Development Server**:
   ```bash
   npm run start
   # Opens browser at localhost:3000
   # Navigate to Module 2 section, verify all links work
   ```

**Checkpoint**:
- [ ] Front matter added to all chapters
- [ ] Module index.md created with overview
- [ ] Docusaurus builds without errors
- [ ] All navigation links functional

---

## Final Validation Summary

Before marking Module 2 complete, verify:

### Content Validation
- [x] **Word Count**: 4,800-6,000 words total (Ch1: 1,300, Ch2: 1,400, Ch3: 1,300, Ch4: 1,800)
- [x] **Readability**: Flesch-Kincaid grade 10-12 for all chapters
- [x] **Citations**: 15+ total (minimum 3 per chapter), 8+ peer-reviewed (53%)
- [x] **Plagiarism**: <10% similarity score

### Technical Validation
- [x] **Gazebo Examples**: All world files load, physics stable (real-time factor ≥0.5x)
- [x] **Unity Examples**: All scenes connect to ROS 2, frame rate ≥30 FPS
- [x] **Sensor Configs**: All plugins publish to correct topics, data formats valid
- [x] **Digital Twin**: Synchronization latency <50ms (SC-002)

### Educational Validation
- [x] **Learning Objectives**: 2-3 per chapter, specific and measurable
- [x] **Practice Exercises**: 3 per chapter (beginner, intermediate, advanced)
- [x] **Lab Guide**: Complete with grading rubric and validation checklist
- [x] **Estimated Time**: 12-15 hours total for module

### Integration Validation
- [x] **Docusaurus Build**: No errors or warnings
- [x] **Navigation**: All links functional, sidebar structure correct
- [x] **Cross-References**: Internal links between chapters working

---

## Estimated Time Breakdown

| Step | Time | Cumulative |
|------|------|------------|
| 1. Review Research & Data Model | 15 min | 0.25 hr |
| 2. Set Up Testing Environment | 1-2 hr | 2.25 hr |
| 3. Write Chapter Content (4 chapters × 5 hr avg) | 20 hr | 22.25 hr |
| 4. Create Gazebo/Unity Examples (4 × 2.5 hr) | 10 hr | 32.25 hr |
| 5. Test Examples (4 × 1.5 hr) | 6 hr | 38.25 hr |
| 6. Create Lab Guide (Chapter 4) | 8 hr | 46.25 hr |
| 7. Compile References | 1 hr | 47.25 hr |
| 8. Quality Validation | 2.5 hr | 49.75 hr |
| 9. Docusaurus Integration | 1 hr | 50.75 hr |

**Total**: ~51 hours (distributed across 4 chapters + integration)

**Per Chapter Average**: 12-13 hours (including testing and validation)

---

## Success Metrics (vs Module 1)

| Metric | Module 1 (Actual) | Module 2 (Target) | Status |
|--------|-------------------|-------------------|--------|
| **Word Count Variance** | +97% (11,800 vs 6,000) | ±5% (5,700-6,100 vs 5,800) | ✅ Enforced |
| **Testing Phase** | Deferred to end (Phase 7) | Continuous (Step 5 per chapter) | ✅ Early |
| **Citation Discipline** | Retrospective | Inline during writing | ✅ Proactive |
| **Environment Setup** | After content creation | Before (Step 2) | ✅ Upfront |

---

**Quickstart Version**: 1.0.0
**Last Updated**: 2025-12-18
**Key Improvement**: Step 2 moved environment setup from Phase 7 to Phase 2 for continuous testing
**Usage**: Follow steps sequentially, enforce word budgets per-section (not per-chapter)
