using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;

/// <summary>
/// RobotController subscribes to ROS 2 /joint_states topic and updates Unity Articulation Bodies.
/// Attach this script to the root GameObject of your robot hierarchy.
/// </summary>
/// <remarks>
/// Requirements:
/// - Robot GameObject hierarchy uses Articulation Bodies (not Rigidbodies)
/// - Joint names in Unity match URDF joint names exactly
/// - ROS-TCP-Connector package installed and ROSConnection active
/// </remarks>
public class RobotController : MonoBehaviour
{
    [Header("ROS 2 Configuration")]
    [Tooltip("ROS 2 topic name to subscribe to")]
    public string jointStateTopic = "/joint_states";

    [Header("Robot Configuration")]
    [Tooltip("Root Articulation Body of the robot (typically base_link)")]
    public ArticulationBody robotRoot;

    [Tooltip("Enable debug logging for joint updates")]
    public bool debugMode = false;

    [Header("Performance Monitoring")]
    [Tooltip("Display latency warning if sync delay exceeds this threshold (seconds)")]
    public float latencyThreshold = 0.05f; // 50ms as per SC-002

    // Private fields
    private ArticulationBody[] joints;
    private Dictionary<string, ArticulationBody> jointMap;
    private int messageCount = 0;
    private float lastMessageTime = 0f;

    void Start()
    {
        // Validate robot root is assigned
        if (robotRoot == null)
        {
            Debug.LogError("RobotController: robotRoot not assigned! Assign the root Articulation Body in Inspector.");
            enabled = false;
            return;
        }

        // Get all Articulation Bodies in robot hierarchy (including children)
        joints = robotRoot.GetComponentsInChildren<ArticulationBody>();
        Debug.Log($"RobotController: Found {joints.Length} Articulation Bodies in robot hierarchy");

        // Build joint name map for fast lookup
        jointMap = new Dictionary<string, ArticulationBody>();
        foreach (var joint in joints)
        {
            // Use GameObject name as joint name (must match URDF joint names)
            string jointName = joint.gameObject.name;
            if (!jointMap.ContainsKey(jointName))
            {
                jointMap[jointName] = joint;
                if (debugMode)
                    Debug.Log($"RobotController: Registered joint '{jointName}'");
            }
            else
            {
                Debug.LogWarning($"RobotController: Duplicate joint name '{jointName}' - skipping");
            }
        }

        // Subscribe to ROS 2 /joint_states topic
        try
        {
            ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>(
                jointStateTopic,
                OnJointStateReceived
            );
            Debug.Log($"RobotController: Subscribed to {jointStateTopic}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"RobotController: Failed to subscribe to {jointStateTopic}: {e.Message}");
            enabled = false;
        }
    }

    /// <summary>
    /// Callback triggered when /joint_states message received from ROS 2
    /// </summary>
    /// <param name="jointStateMsg">ROS 2 JointState message containing joint positions/velocities</param>
    void OnJointStateReceived(JointStateMsg jointStateMsg)
    {
        messageCount++;
        float currentTime = Time.time;

        // Latency monitoring (requires synchronized clocks)
        double rosTime = jointStateMsg.header.stamp.sec + jointStateMsg.header.stamp.nanosec * 1e-9;
        double unityTime = Time.time; // Approximate - not synchronized with ROS time
        double latency = unityTime - rosTime;

        if (latency > latencyThreshold)
        {
            Debug.LogWarning($"RobotController: High latency detected: {latency * 1000:F1}ms (threshold: {latencyThreshold * 1000:F0}ms)");
        }

        if (debugMode && messageCount % 100 == 0) // Log every 100 messages to avoid spam
        {
            Debug.Log($"RobotController: Received message #{messageCount}, {jointStateMsg.name.Length} joints, latency: {latency * 1000:F1}ms");
        }

        // Update joint positions
        for (int i = 0; i < jointStateMsg.name.Length; i++)
        {
            string jointName = jointStateMsg.name[i];

            // Check if position array has corresponding index
            if (i >= jointStateMsg.position.Length)
            {
                Debug.LogWarning($"RobotController: Joint '{jointName}' has no position data (index {i} out of bounds)");
                continue;
            }

            double position = jointStateMsg.position[i]; // Radians for revolute, meters for prismatic

            // Find matching Unity Articulation Body
            if (jointMap.TryGetValue(jointName, out ArticulationBody joint))
            {
                // Get current drive configuration
                var drive = joint.xDrive;

                // Convert radians to degrees for Unity (Unity uses degrees internally)
                float targetDegrees = (float)(position * Mathf.Rad2Deg);

                // Set target position
                drive.target = targetDegrees;

                // Apply updated drive configuration
                joint.xDrive = drive;

                if (debugMode && i < 3) // Log first 3 joints to avoid spam
                {
                    Debug.Log($"RobotController: Updated '{jointName}' to {targetDegrees:F2}° ({position:F3} rad)");
                }
            }
            else
            {
                // Joint name mismatch - common issue
                if (messageCount == 1) // Only warn once per session
                {
                    Debug.LogWarning($"RobotController: Joint '{jointName}' from ROS not found in Unity hierarchy. " +
                                     "Ensure GameObject names match URDF joint names exactly.");
                }
            }
        }

        lastMessageTime = currentTime;
    }

    /// <summary>
    /// Display connection statistics in Unity Editor
    /// </summary>
    void OnGUI()
    {
        if (debugMode)
        {
            GUILayout.Label($"RobotController Status");
            GUILayout.Label($"Topic: {jointStateTopic}");
            GUILayout.Label($"Messages Received: {messageCount}");
            GUILayout.Label($"Last Update: {Time.time - lastMessageTime:F2}s ago");
            GUILayout.Label($"Joints Mapped: {jointMap.Count} / {joints.Length}");
        }
    }

    /// <summary>
    /// Validate robot configuration on script load (Unity Editor only)
    /// </summary>
    void OnValidate()
    {
        // Warn if robotRoot not assigned
        if (robotRoot == null)
        {
            Debug.LogWarning("RobotController: robotRoot is not assigned. Assign the root Articulation Body of your robot.");
        }

        // Validate latency threshold
        if (latencyThreshold <= 0)
        {
            Debug.LogWarning("RobotController: latencyThreshold must be > 0. Setting to default 0.05s (50ms).");
            latencyThreshold = 0.05f;
        }
    }
}

/*
 * Usage Instructions:
 *
 * 1. Attach to Robot Root GameObject:
 *    - Select root GameObject of robot hierarchy (e.g., "HumanoidRobot")
 *    - Inspector > Add Component > RobotController
 *
 * 2. Assign Robot Root:
 *    - Drag root GameObject's Articulation Body into "Robot Root" field
 *
 * 3. Configure Topic Name (if different):
 *    - Default: /joint_states
 *    - For namespaced robot: /robot1/joint_states
 *
 * 4. Enable Debug Mode (optional):
 *    - Check "Debug Mode" to see joint update logs in Console
 *
 * 5. Run:
 *    - Start ROS-TCP-Endpoint: ros2 run ros_tcp_endpoint default_server_endpoint
 *    - Start Gazebo with robot: ros2 launch your_package gazebo.launch.py
 *    - Press Play in Unity Editor
 *    - Expected: Robot joints move in sync with Gazebo simulation
 *
 * Troubleshooting:
 * - "Joint 'X' not found": GameObject name doesn't match URDF joint name
 *   Solution: Rename Unity GameObjects to match URDF exactly, or modify URDF
 *
 * - "High latency detected": Network delay or Unity frame rate too low
 *   Solution: Run ROS-TCP-Endpoint on same machine, increase Unity target FPS to 60
 *
 * - Joints don't move: xDrive settings incorrect
 *   Solution: Increase xDrive.stiffness (try 10000), reduce damping (try 100)
 *
 * - Jittery motion: xDrive stiffness too high
 *   Solution: Reduce stiffness to 5000, increase damping to 500
 */
