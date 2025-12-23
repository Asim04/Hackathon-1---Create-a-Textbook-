# Chapter 1: ROS 2 Fundamentals - Code Examples

This directory contains working Python code examples demonstrating ROS 2 fundamentals: nodes, topics, and parameters.

---

## Prerequisites

- **Operating System**: Ubuntu 22.04 LTS
- **ROS 2 Version**: Humble Hawksbill
- **Python**: 3.10 or later

### Installation Check

Verify your ROS 2 installation:
```bash
# Check ROS 2 version
ros2 --version
# Expected: ros2 cli version 0.X.X

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Verify Python version
python3 --version
# Expected: Python 3.10.X or later
```

If ROS 2 is not installed, see [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html).

---

## Code Examples

### 1. publisher_node.py - Simple Publisher

**Purpose**: Demonstrates creating a ROS 2 node that publishes String messages to `/hello_topic` every second.

**Concepts Covered**:
- Node creation with `rclpy`
- Creating a publisher with `create_publisher()`
- Using timers for periodic callbacks
- Publishing messages to a topic

**Run**:
```bash
python3 publisher_node.py
```

**Expected Output**:
```
[INFO] [publisher_node]: Publisher node started. Publishing to /hello_topic every 1 second.
[INFO] [publisher_node]: Publishing: "Hello ROS 2! Message #0"
[INFO] [publisher_node]: Publishing: "Hello ROS 2! Message #1"
[INFO] [publisher_node]: Publishing: "Hello ROS 2! Message #2"
...
```

**Stop**: Press `Ctrl+C`

---

### 2. subscriber_node.py - Simple Subscriber

**Purpose**: Demonstrates creating a ROS 2 node that subscribes to `/hello_topic` and prints received messages.

**Concepts Covered**:
- Creating a subscriber with `create_subscription()`
- Callback functions for message processing
- Asynchronous message reception

**Run** (in a separate terminal from publisher):
```bash
# Terminal 1 (keep running)
python3 publisher_node.py

# Terminal 2 (run subscriber)
python3 subscriber_node.py
```

**Expected Output** (in Terminal 2):
```
[INFO] [subscriber_node]: Subscriber node started. Listening to /hello_topic.
[INFO] [subscriber_node]: Received: "Hello ROS 2! Message #5"
[INFO] [subscriber_node]: Received: "Hello ROS 2! Message #6"
[INFO] [subscriber_node]: Received: "Hello ROS 2! Message #7"
...
```

**Stop**: Press `Ctrl+C` in both terminals

---

## Testing Publisher-Subscriber Communication

### Basic Test
1. Start publisher: `python3 publisher_node.py`
2. Start subscriber in new terminal: `python3 subscriber_node.py`
3. Verify subscriber receives messages from publisher

### Using ROS 2 Command-Line Tools

```bash
# List all active nodes
ros2 node list
# Should show: /publisher_node and /subscriber_node

# List all active topics
ros2 topic list
# Should show: /hello_topic (among others)

# Echo messages on a topic (alternative to subscriber)
ros2 topic echo /hello_topic

# Check publish rate
ros2 topic hz /hello_topic
# Should show: ~1.0 Hz

# Get topic info
ros2 topic info /hello_topic
# Shows: message type, publishers, subscribers
```

---

## Troubleshooting

### Issue: "ModuleNotFoundError: No module named 'rclpy'"

**Cause**: ROS 2 environment not sourced

**Solution**:
```bash
source /opt/ros/humble/setup.bash
# Add to ~/.bashrc to source automatically:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

---

### Issue: "Node name 'publisher_node' already exists"

**Cause**: Node with same name already running

**Solution**:
```bash
# List running nodes
ros2 node list

# Kill the existing node
pkill -f publisher_node

# Or use unique node names when launching
```

---

### Issue: Subscriber doesn't receive messages

**Possible Causes**:
1. **Publisher not running**: Verify with `ros2 node list`
2. **Topic name mismatch**: Check with `ros2 topic list`
3. **Network issues**: Ensure both nodes on same ROS_DOMAIN_ID

**Debugging**:
```bash
# Check if messages are being published
ros2 topic echo /hello_topic

# Check topic info (should show 1 publisher, 1 subscriber)
ros2 topic info /hello_topic
```

---

## Extensions

Try these modifications to deepen your understanding:

1. **Change Message Type**: Modify publisher to send Int32 instead of String
2. **Multiple Publishers**: Run two publisher nodes publishing to the same topic
3. **Message Filtering**: Modify subscriber to only print messages containing a specific word
4. **Rate Control**: Use parameters to make publish rate configurable (see parameterized_node.py example in chapter text)

---

**Code Tested On**: Ubuntu 22.04, ROS 2 Humble, Python 3.10
**Last Updated**: 2025-12-17
**License**: Educational use
