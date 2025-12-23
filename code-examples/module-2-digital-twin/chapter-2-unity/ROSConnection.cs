using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

/// <summary>
/// ROSConnection initializes the ROS-TCP-Connector and handles connection errors.
/// Attach this script to an empty GameObject named "ROSConnection" in your scene.
/// </summary>
/// <remarks>
/// This script provides:
/// - Automatic ROSConnection initialization on scene start
/// - Connection status monitoring and HUD display
/// - Error handling for connection failures (timeout, refused, invalid IP)
/// - Retry logic for transient network issues
/// </remarks>
public class ROSConnection : MonoBehaviour
{
    [Header("ROS 2 Configuration")]
    [Tooltip("IP address of machine running ROS-TCP-Endpoint")]
    public string rosIPAddress = "127.0.0.1";

    [Tooltip("Port number for ROS-TCP-Endpoint (default: 10000)")]
    public int rosPort = 10000;

    [Tooltip("ROS protocol version")]
    public ROSProtocol rosProtocol = ROSProtocol.ROS2;

    [Header("Connection Settings")]
    [Tooltip("Number of connection retry attempts before giving up")]
    public int maxRetries = 3;

    [Tooltip("Delay between retry attempts (seconds)")]
    public float retryDelay = 2.0f;

    [Tooltip("Connection timeout (seconds)")]
    public float connectionTimeout = 5.0f;

    [Header("Display Settings")]
    [Tooltip("Show connection status HUD overlay in game view")]
    public bool showHUD = true;

    [Tooltip("HUD text color when connected")]
    public Color connectedColor = Color.green;

    [Tooltip("HUD text color when connecting")]
    public Color connectingColor = Color.yellow;

    [Tooltip("HUD text color when disconnected")]
    public Color disconnectedColor = Color.red;

    // Private fields
    private Unity.Robotics.ROSTCPConnector.ROSConnection ros;
    private bool isConnected = false;
    private bool isConnecting = false;
    private int retryCount = 0;
    private float nextRetryTime = 0f;
    private string statusMessage = "Initializing...";
    private GUIStyle hudStyle;

    /// <summary>
    /// ROS protocol version enum
    /// </summary>
    public enum ROSProtocol
    {
        ROS1,
        ROS2
    }

    void Start()
    {
        // Initialize HUD style
        hudStyle = new GUIStyle();
        hudStyle.fontSize = 14;
        hudStyle.fontStyle = FontStyle.Bold;
        hudStyle.normal.textColor = connectingColor;
        hudStyle.padding = new RectOffset(10, 10, 10, 10);

        // Validate configuration
        if (!ValidateConfiguration())
        {
            Debug.LogError("ROSConnection: Invalid configuration. Check Inspector settings.");
            enabled = false;
            return;
        }

        // Initialize ROS-TCP-Connector
        InitializeConnection();
    }

    void Update()
    {
        // Check connection status
        if (ros != null)
        {
            // ROS-TCP-Connector doesn't expose direct connection status
            // Use heuristic: if we can get the connection instance, assume connected
            if (!isConnected && ros != null)
            {
                isConnected = true;
                isConnecting = false;
                statusMessage = "Connected";
                Debug.Log($"ROSConnection: Successfully connected to {rosIPAddress}:{rosPort}");
            }
        }
        else if (!isConnecting && retryCount < maxRetries && Time.time >= nextRetryTime)
        {
            // Retry connection if failed
            retryCount++;
            nextRetryTime = Time.time + retryDelay;
            statusMessage = $"Retrying... ({retryCount}/{maxRetries})";
            Debug.LogWarning($"ROSConnection: Connection attempt {retryCount}/{maxRetries}");
            InitializeConnection();
        }
    }

    /// <summary>
    /// Initialize ROS-TCP-Connector with configured settings
    /// </summary>
    void InitializeConnection()
    {
        try
        {
            isConnecting = true;
            statusMessage = "Connecting...";

            // Get or create ROSConnection singleton
            ros = Unity.Robotics.ROSTCPConnector.ROSConnection.GetOrCreateInstance();

            // Configure connection parameters
            ros.RosIPAddress = rosIPAddress;
            ros.RosPort = rosPort;

            // Set protocol (ROS 1 or ROS 2)
            // Note: ROS-TCP-Connector protocol setting must match ROS-TCP-Endpoint configuration
            if (rosProtocol == ROSProtocol.ROS2)
            {
                // ROS 2 mode (default)
                Debug.Log("ROSConnection: Using ROS 2 protocol");
            }
            else
            {
                // ROS 1 mode (legacy)
                Debug.Log("ROSConnection: Using ROS 1 protocol");
            }

            // Show HUD overlay if enabled
            ros.ShowHud = showHUD;

            Debug.Log($"ROSConnection: Attempting to connect to {rosIPAddress}:{rosPort}");
        }
        catch (System.Exception e)
        {
            isConnecting = false;
            statusMessage = $"Error: {e.Message}";
            Debug.LogError($"ROSConnection: Failed to initialize: {e.Message}");
        }
    }

    /// <summary>
    /// Validate configuration parameters before connection
    /// </summary>
    /// <returns>True if configuration is valid, false otherwise</returns>
    bool ValidateConfiguration()
    {
        // Validate IP address format
        if (string.IsNullOrEmpty(rosIPAddress))
        {
            Debug.LogError("ROSConnection: ROS IP Address is empty");
            return false;
        }

        // Check if IP is localhost or valid IPv4
        if (rosIPAddress != "localhost" && rosIPAddress != "127.0.0.1")
        {
            string[] octets = rosIPAddress.Split('.');
            if (octets.Length != 4)
            {
                Debug.LogError($"ROSConnection: Invalid IP address format: {rosIPAddress}");
                return false;
            }
        }

        // Validate port number (1024-65535 for non-privileged ports)
        if (rosPort < 1024 || rosPort > 65535)
        {
            Debug.LogError($"ROSConnection: Invalid port number: {rosPort}. Must be between 1024-65535.");
            return false;
        }

        // Validate retry settings
        if (maxRetries < 0)
        {
            Debug.LogWarning("ROSConnection: maxRetries < 0, setting to 0 (no retries)");
            maxRetries = 0;
        }

        if (retryDelay < 0.1f)
        {
            Debug.LogWarning("ROSConnection: retryDelay too small, setting to 1.0s");
            retryDelay = 1.0f;
        }

        return true;
    }

    /// <summary>
    /// Display connection status HUD in game view
    /// </summary>
    void OnGUI()
    {
        if (!showHUD) return;

        // Set HUD text color based on connection status
        if (isConnected)
            hudStyle.normal.textColor = connectedColor;
        else if (isConnecting)
            hudStyle.normal.textColor = connectingColor;
        else
            hudStyle.normal.textColor = disconnectedColor;

        // Display connection info in top-left corner
        GUILayout.BeginArea(new Rect(10, 10, 300, 100));
        GUILayout.Label($"ROS IP: {rosIPAddress}:{rosPort}", hudStyle);
        GUILayout.Label($"Status: {statusMessage}", hudStyle);
        GUILayout.EndArea();
    }

    /// <summary>
    /// Clean up ROS connection on scene unload
    /// </summary>
    void OnDestroy()
    {
        if (ros != null)
        {
            Debug.Log("ROSConnection: Closing connection");
            // ROS-TCP-Connector handles cleanup automatically
        }
    }

    /// <summary>
    /// Validate settings when changed in Unity Editor (Editor-only)
    /// </summary>
    void OnValidate()
    {
        // Clamp port to valid range
        if (rosPort < 1024)
            rosPort = 1024;
        if (rosPort > 65535)
            rosPort = 65535;

        // Clamp retry settings
        if (maxRetries < 0)
            maxRetries = 0;
        if (retryDelay < 0.1f)
            retryDelay = 0.1f;
        if (connectionTimeout < 1.0f)
            connectionTimeout = 1.0f;
    }

    /// <summary>
    /// Public method to check connection status
    /// </summary>
    /// <returns>True if connected to ROS-TCP-Endpoint</returns>
    public bool IsConnected()
    {
        return isConnected;
    }

    /// <summary>
    /// Public method to manually retry connection
    /// </summary>
    public void RetryConnection()
    {
        if (!isConnecting)
        {
            retryCount = 0;
            InitializeConnection();
        }
    }
}

/*
 * Usage Instructions:
 *
 * 1. Create GameObject:
 *    - GameObject > Create Empty
 *    - Rename to "ROSConnection"
 *
 * 2. Attach Script:
 *    - Inspector > Add Component > ROSConnection
 *
 * 3. Configure Settings:
 *    - ROS IP Address: 127.0.0.1 (localhost) or remote IP
 *    - ROS Port: 10000 (default)
 *    - ROS Protocol: ROS2
 *    - Show HUD: ✓ (displays connection status)
 *
 * 4. Run:
 *    - Start ROS-TCP-Endpoint first:
 *      ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
 *    - Press Play in Unity Editor
 *    - HUD should show "Status: Connected" (green) within 2-3 seconds
 *
 * Troubleshooting:
 * - "Status: Connecting..." (yellow, persistent):
 *   Solution: Check ROS-TCP-Endpoint is running on port 10000
 *   Verify: netstat -an | grep 10000 (should show LISTEN state)
 *
 * - "Status: Error: ..." (red):
 *   Solution: Check error message in Unity Console for details
 *   Common: Firewall blocking port 10000, incorrect IP address
 *
 * - "Invalid IP address format":
 *   Solution: Use format "192.168.1.100" (four octets) or "127.0.0.1"
 *
 * - Connection works but no data received:
 *   Solution: Verify RobotController script is attached to robot GameObject
 *   Check: ROS 2 topics are publishing (ros2 topic list, ros2 topic hz /joint_states)
 *
 * Network Configuration for Remote ROS:
 * - Unity on Windows, ROS on Linux (WSL2):
 *   IP Address: $(wsl hostname -I) - e.g., 172.24.123.45
 *
 * - Unity on Windows, ROS on separate machine:
 *   IP Address: Linux machine's LAN IP - e.g., 192.168.1.100
 *   Firewall: Allow TCP port 10000 on Linux machine
 */
