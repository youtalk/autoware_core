# Using Zenoh RMW with Autoware Core

## Overview

Autoware Core is RMW-agnostic. All packages use standard rclcpp/rclpy
abstractions and can be used with any RMW implementation supported by
ROS 2.

## Jazzy: rmw_zenoh_cpp

ROS 2 Jazzy includes `rmw_zenoh_cpp` as a Tier 1 RMW implementation.
To use Zenoh as the middleware for Autoware Core:

### Installation

```bash
sudo apt install ros-jazzy-rmw-zenoh-cpp
```

### Usage

Set the environment variable before launching any Autoware node:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

Or per-launch:

```bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 launch autoware_launch autoware.launch.xml
```

### Zenoh Router

Unlike CycloneDDS (multicast-based), Zenoh uses a router for discovery.
Start the router before launching nodes:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### Configuration

Zenoh configuration is via `DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5` and
`DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5`. These are installed with the
`rmw_zenoh_cpp` package at:

```text
/opt/ros/jazzy/share/rmw_zenoh_cpp/config/
```

Custom config:

```bash
export ZENOH_SESSION_CONFIG_URI=/path/to/custom_config.json5
```

## QoS Compatibility

Autoware Core defines QoS policies in
`common/autoware_component_interface_specs/`. These are RMW-agnostic
and work with Zenoh. The QoS policies used:

- RELIABLE + TRANSIENT_LOCAL: state topics (route, map, initialization)
- RELIABLE + VOLATILE: streaming data (trajectory, control, perception)

Zenoh supports all standard QoS policies.

## Known Considerations

1. **Discovery**: Zenoh uses router-based discovery instead of multicast.
   Ensure `rmw_zenohd` is running before nodes start.
2. **Agnocast**: The Agnocast zero-copy transport is independent of the
   RMW layer. It operates at the intra-process level and is compatible
   with any RMW implementation.
3. **Humble**: `rmw_zenoh_cpp` is not available as a binary package for
   Humble. Zenoh usage requires Jazzy or later.
