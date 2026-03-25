# mapviz_mavros_plugins

A mapviz plugin for visualizing MAVROS waypoint lists on a 2D map. Displays flight waypoints as colored circles with the current waypoint highlighted, and optionally draws connection lines between sequential waypoints.

## Features

- Subscribes to `mavros_msgs/msg/WaypointList` topics and renders global-frame waypoints on the mapviz canvas
- Highlights the current active waypoint with a distinct color and larger ring
- Draws connection lines between sequential waypoints (toggleable)
- Fully configurable colors, sizes, and line thickness via the Qt config panel
- Supports saving/loading configuration via mapviz YAML config files
- Configurable ROS 2 QoS settings (history, reliability, durability, depth)

## Configuration

| Setting | Default | Description |
|---------|---------|-------------|
| **Topic** | *(empty)* | ROS 2 topic publishing `mavros_msgs/msg/WaypointList` |
| **Waypoint Color** | Green | Color for regular waypoint circles and connection lines |
| **Current WP Color** | Yellow | Color for the current waypoint highlight |
| **Waypoint Size** | 3.0 m | Radius of waypoint circles in meters (0.5 - 100.0) |
| **Show Connections** | On | Toggle connection lines between waypoints |
| **Line Thickness** | 2 px | Thickness of circle outlines and connection lines (1 - 20) |

## Building

This package is built as part of the desktop colcon workspace. From inside the docker container:

```sh
colcon build --symlink-install --packages-select mapviz_mavros_plugins
```

### Dependencies

- `mapviz` - visualization framework
- `mavros_msgs` - MAVROS message definitions
- `swri_transform_util` - coordinate frame transforms
- `pluginlib` - dynamic plugin loading
- Qt5 (Core, Gui, OpenGL, Widgets)

## Usage

1. Launch mapviz
2. Add the **mapviz_plugins/waypoint_list** plugin from the plugin list
3. Set the topic to your MAVROS waypoint list topic (e.g. `/mavros/mission/waypoints`)
4. Waypoints will appear on the map as the topic publishes data

The plugin only visualizes waypoints in global coordinate frames (`FRAME_GLOBAL`, `FRAME_GLOBAL_REL_ALT`, `FRAME_GLOBAL_TERRAIN_ALT`). Local-frame waypoints are filtered out.
