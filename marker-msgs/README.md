# marker_msgs

## Overview

`marker_msgs` is a ROS 2 message package that defines standardized messages for communicating information about visual marker detection output (such as ArUco, AprilTag, QR codes, or fiducials).
These messages enable interoperability between perception, localization, and visualization components in fudicial marker based applications.

## Features

- **Custom messages for markers:**
  - `MarkerPose.msg`
    ```uint32 id
    string location
    geometry_msgs/Pose pose
    ```
  - `MarkerPoseArray.msg`
    ```std_msgs/Header header
    marker_msgs/MarkerPose[] markers
    ```
- Compatible with standard ROS 2 message workflows.

## Building and Installing

1. **Clone into your ROS 2 workspace:**
    ```sh
    cd ~/ros2_ws/src
    # The repo will be updated to the latest version
    git clone https://gitlab.com/schaffler-nav/marker-msgs
    # marker_msgs is can be included under other messages repository 
    ```

2. **Build the workspace:**
    ```sh
    cd ~/ros2_ws
    colcon build --packages-select marker_msgs
    ```

3. **Source your workspace:**
    ```sh
    source install/setup.bash
    ```

## Usage in Your ROS 2 Package

To use `marker_msgs` in your own package (for publishing or subscribing):

### 1. Add Dependency in package.xml
```xml
<depend>marker_msgs</depend>
```

### 2. Find Package in CMakeLists.txt
```cmake
find_package(marker_msgs REQUIRED)
```

### 3. Add to `ament_target_dependencies`
```cmake
ament_target_dependencies(your_node
  rclcpp
  marker_msgs
  # ...other dependencies
)
```

### 4. Include Message Headers in Your Code
```cpp
#include "marker_msgs/msg/marker_pose.hpp"
#include "marker_msgs/msg/marker_pose_array.hpp"
```

### 5. Publish or Subscribe to Marker Messages

#### Publisher Example (C++):
```cpp
auto pub = node->create_publisher<marker_msgs::msg::MarkerPoseArray>("detected_markers", 10);

marker_msgs::msg::MarkerArray msg;
// ...fill msg...
pub->publish(msg);
```

#### Subscriber Example (C++):
```cpp
auto sub = node->create_subscription<marker_msgs::msg::MarkerPose>(
  "detected_markers", 10,
  [](const marker_msgs::msg::MarkerArray::SharedPtr msg) {
    // ...process markers...
  });
```

## Message Reference

- **Marker**: Represents a single detected marker (ID, pose, type, confidence, etc.).
- **MarkerArray**: Array of `Marker` messages.

**Full message definitions:**
See the `msg/` directory in the package source for `.msg` files.


## Troubleshooting

- Ensure `marker_msgs` is built and sourced in your workspace before building dependent packages.
- If using Python, import via:
  ```python
  from marker_msgs.msg import MarkerPose, MarkerPoseArray
  ```

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) (if available) or open a pull request on the upstream repository.

## License

See the `LICENSE` file for details.

**For more information:**
This package is written and maintained by [Uma Ramu](mailto:ramu_uma@artc.a-star.edu.sg)