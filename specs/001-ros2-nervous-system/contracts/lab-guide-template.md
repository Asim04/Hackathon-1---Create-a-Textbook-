# Hands-On Lab: [Lab Title]

**Module**: [Module Name]
**Estimated Duration**: [2-3] hours
**Difficulty**: [Beginner | Intermediate | Advanced]
**Chapter**: [Usually Chapter 4 - Integrative Lab]

---

## Lab Overview

[2-3 paragraphs describing:
- What students will build/accomplish in this lab
- Why this lab is important (integrates concepts from prior chapters)
- Real-world relevance or applications of the skills practiced]

---

## Learning Objectives

By completing this lab, you will be able to:
- [Objective 1 - specific, measurable skill demonstrated in lab]
- [Objective 2 - integration of concepts from multiple chapters]
- [Objective 3 - practical application or system-building skill]
- [Objective 4 - debugging or problem-solving skill if applicable]

---

## Prerequisites

### Knowledge Prerequisites
- [ ] Completed Chapter 1: [Topic]
- [ ] Completed Chapter 2: [Topic]
- [ ] Completed Chapter 3: [Topic]
- [ ] Understanding of [key concept from prior chapters]

### Software Prerequisites
- [ ] Ubuntu 22.04 LTS installed (or compatible Linux distribution)
- [ ] ROS 2 Humble Hawksbill installed and configured
- [ ] Python 3.10 or later
- [ ] [Other required software/tools]

**Verification Commands**:
```bash
# Verify ROS 2 installation
ros2 --version
# Expected output: ros2 cli version X.X.X

# Verify Python version
python3 --version
# Expected output: Python 3.10.X or later

# [Additional verification commands]
```

---

## Materials

### Software & Tools
- **ROS 2 Humble**: Robot Operating System framework
- **Gazebo Classic 11**: Physics simulation environment
- **RViz2**: Visualization tool
- **Python 3.10+**: Programming language for control scripts
- **[Other tools/libraries]**

### Starter Code
- **Repository**: [GitHub URL or path to starter code]
- **Files Provided**:
  - `[starter_file_1.ext]`: [Description]
  - `[starter_file_2.ext]`: [Description]
  - `README.md`: Setup instructions

**Download Starter Code**:
```bash
# Clone or download starter code
cd ~/[workspace]
[git clone command or download instructions]
```

### Hardware (if applicable)
- **None**: This lab uses simulation only
  OR
- **Required**: [List any physical hardware needed]

---

## Lab Procedures

### Part 1: [Initial Setup and Project Initialization]

**Objective**: Set up the workspace and verify all tools are functioning

**Steps**:

1. **Create project workspace**
   ```bash
   mkdir -p ~/[project_name]/[subdirs]
   cd ~/[project_name]
   ```

   **Expected Outcome**: Directory structure created successfully

   **✅ Checkpoint**: Run `ls -la` and verify directories exist

2. **Initialize ROS 2 workspace**
   ```bash
   [command to initialize workspace]
   ```

   **Expected Outcome**: [What should happen]

   **✅ Checkpoint**: [How to verify success]

3. **[Additional setup steps]**
   [Continue with numbered steps]

   **Expected Outcome**: [Result]

   **✅ Checkpoint**: [Verification method]

---

### Part 2: [Main Development/Build Activity]

**Objective**: [What student will create/implement in this part]

**Steps**:

1. **[Step description]**
   ```python
   # [Code or commands]
   ```

   **Explanation**: [Why this step is necessary, what concept it demonstrates]

   **Expected Outcome**: [What should happen]

   **✅ Checkpoint**: [Verification command or observation]

2. **[Next step]**
   [Continue with clear, sequential steps]

[Continue for all major activities in Part 2]

---

### Part 3: [Testing and Validation]

**Objective**: Verify system functionality and debug any issues

**Steps**:

1. **Launch simulation environment**
   ```bash
   [launch command]
   ```

   **Expected Outcome**: [Simulation window opens, robot appears, etc.]

   **✅ Checkpoint**: Verify [specific observable behavior]

2. **Run control scripts**
   ```bash
   [command to run scripts]
   ```

   **Expected Outcome**: [Robot moves, sensors publish data, etc.]

   **✅ Checkpoint**: [How to verify correct behavior]

3. **Monitor sensor data**
   ```bash
   [command to view sensor topics]
   ```

   **Expected Outcome**: [Data stream visible with expected values]

   **✅ Checkpoint**: [Verification method]

---

### Part 4: [Integration and Advanced Features] (Optional if time permits)

**Objective**: [Extend basic functionality with additional features]

[Similar structure to previous parts]

---

## Expected Outcomes

Upon successful completion, you should have:

- [ ] **Outcome 1**: [Specific deliverable - e.g., "Working URDF model that loads in RViz without errors"]
- [ ] **Outcome 2**: [Specific functionality - e.g., "Robot arm moves to 3 target positions with <5° error"]
- [ ] **Outcome 3**: [Integration success - e.g., "Sensor data publishes at 10Hz and can be visualized"]
- [ ] **Outcome 4**: [System behavior - e.g., "Complete system runs for 5 minutes without crashes"]

**Demonstration Video** (Optional):
[If applicable, describe what a demo video should show to prove completion]

---

## Troubleshooting

### Common Issues

| Issue | Symptoms | Likely Cause | Solution |
|-------|----------|--------------|----------|
| **[Issue 1 Name]** | [What student observes] | [Root cause] | **Fix**: [Step-by-step solution with commands]<br>1. [First step]<br>2. [Second step]<br>3. [Verification] |
| **[Issue 2 Name]** | [Symptoms] | [Cause] | **Fix**: [Solution steps] |
| **[Issue 3 Name]** | [Symptoms] | [Cause] | **Fix**: [Solution steps] |
| **[Issue 4 Name]** | [Symptoms] | [Cause] | **Fix**: [Solution steps] |

### Debugging Strategies

**If [category of problems] occur**:
1. [First diagnostic step]
2. [Second diagnostic step]
3. [Where to find logs or error messages]
4. [How to isolate the problem]

**If [another category] occurs**:
[Similar debugging approach]

### Getting Help

- **Lab TA/Instructor**: [Contact information or office hours]
- **Discussion Forum**: [Link to course forum or discussion board]
- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **[Other resources]**

---

## Extensions (Optional Challenges)

Completed the basic lab? Try these advanced challenges:

### Extension 1: [Enhancement Name] (Difficulty: Intermediate)

**Goal**: [What this adds to the base lab]

**Task**: [Specific modification or addition]

**Hints**:
- [Hint 1 - strategic guidance]
- [Hint 2 - relevant documentation or concept]

**Evaluation**: [How to verify successful implementation]

---

### Extension 2: [Advanced Feature] (Difficulty: Advanced)

**Goal**: [Learning objective]

**Task**: [Complex modification or new feature]

**Hints**:
- [High-level guidance]
- [Suggested approach or resources]

**Evaluation**: [Success criteria]

---

### Extension 3: [Creative Challenge] (Difficulty: Advanced)

**Goal**: [Open-ended exploration]

**Task**: [Loosely defined challenge allowing creativity]

**Hints**:
- [General direction]
- [Relevant concepts to apply]

**Evaluation**: [Qualitative assessment criteria]

---

## Submission Requirements (If Graded)

### Required Artifacts

1. **Source Code**
   - [ ] All Python scripts (`.py` files)
   - [ ] URDF robot model files (`.urdf` files)
   - [ ] Launch files (`.launch.py` files)
   - [ ] README.md with setup and run instructions

2. **Documentation**
   - [ ] Screenshot or video demonstrating working system
   - [ ] Brief report (1-2 pages) addressing reflection questions below

3. **Submission Method**
   - **Where**: [Learning management system, GitHub repository, email, etc.]
   - **Format**: [ZIP file, Git repository, etc.]
   - **Deadline**: [Date and time]
   - **Naming Convention**: `[LastName]_[FirstName]_Module[N]_Lab.[ext]`

### Reflection Questions

Answer these questions in your lab report (2-3 sentences each):

1. **Concept Integration**: How did this lab integrate concepts from Chapters 1-3? Provide specific examples.

2. **Challenges Faced**: What was the most challenging part of this lab? How did you overcome it?

3. **Real-World Application**: Describe one real-world robotics application where the skills practiced in this lab would be valuable.

4. **Extensions** (if attempted): Which extension(s) did you complete? What did you learn from implementing them?

---

## Lab Summary

**Key Skills Practiced**:
- [Skill 1 - e.g., "Creating and debugging URDF robot models"]
- [Skill 2 - e.g., "Integrating Python control logic with ROS 2 topics"]
- [Skill 3 - e.g., "Testing sensor data flow in simulation environment"]
- [Skill 4 - e.g., "Troubleshooting communication between ROS 2 nodes"]

**Connection to Module Goals**:
[Paragraph explaining how this lab demonstrates mastery of Module 1 learning objectives and prepares students for future modules]

---

**Lab Metadata**:
- Tested On: Ubuntu 22.04, ROS 2 Humble, Python 3.10, Gazebo Classic 11
- Estimated Completion Time: [X] hours (varies by student experience)
- Success Rate: [If known from prior offerings]
- Last Updated: [Date]

---

**Navigation**:
- Previous Chapter: [Link]
- Module Home: [Link to module index]
- Next Module: [Link to Module 2 or course home]
