---
id: module3-chapter1
title: Introduction to NVIDIA Isaac
sidebar_label: Chapter 1 - NVIDIA Isaac
---

# Introduction to NVIDIA Isaac™

## What is NVIDIA Isaac?

NVIDIA Isaac is a comprehensive robotics platform that combines simulation, AI frameworks, and hardware acceleration to enable next-generation autonomous machines.

### Isaac Platform Components

```
┌──────────────────────────────────────────────┐
│         NVIDIA Isaac Platform                │
├──────────────────────────────────────────────┤
│  Isaac Sim       │ Photorealistic simulation │
│  Isaac SDK       │ Robotics libraries        │
│  Isaac ROS       │ GPU-accelerated ROS nodes │
│  Isaac Manipulator│ Manipulation stack       │
│  Isaac AMR       │ Autonomous mobile robots  │
└──────────────────────────────────────────────┘
```

### Why NVIDIA Isaac?

#### 1. GPU Acceleration
- **PhysX 5**: Hardware-accelerated physics
- **RTX Rendering**: Ray-traced photorealism
- **Tensor Cores**: AI inference acceleration
- **CUDA**: Parallel processing

#### 2. AI Integration
- **Pre-trained Models**: Object detection, segmentation
- **TensorRT**: Optimized inference
- **TAO Toolkit**: Transfer learning
- **DeepStream**: Video analytics

#### 3. Scalability
- **Distributed Simulation**: Multi-GPU support
- **Cloud Deployment**: Omniverse Cloud
- **Fleet Management**: Multi-robot coordination

## Isaac Sim

Isaac Sim is a robotics simulation application built on NVIDIA Omniverse.

### Key Features

**Rendering:**
- ✅ RTX ray tracing and path tracing
- ✅ Photorealistic materials (MDL)
- ✅ Physical lighting and cameras
- ✅ Real-time global illumination

**Physics:**
- ✅ PhysX 5 for rigid bodies
- ✅ Soft body dynamics
- ✅ Fluid simulation
- ✅ Particle systems

**Sensors:**
- ✅ RGB/Depth cameras
- ✅ Lidar (2D/3D)
- ✅ Ultrasonic sensors
- ✅ Contact sensors
- ✅ IMU with realistic noise

**ROS Integration:**
- ✅ ROS 1 and ROS 2 support
- ✅ Publish/subscribe to topics
- ✅ Service calls
- ✅ Action servers

### System Requirements

**Minimum:**
- GPU: RTX 2070 or better
- RAM: 32 GB
- Storage: 50 GB SSD
- OS: Ubuntu 20.04/22.04 or Windows 10/11

**Recommended:**
- GPU: RTX 3080 or better
- RAM: 64 GB
- Storage: 100 GB NVMe SSD
- Multi-core CPU

## Installation

### Isaac Sim Installation

```bash
# 1. Install Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage

# 2. In Omniverse Launcher:
# - Go to Exchange tab
# - Find "Isaac Sim"
# - Click Install

# 3. Launch Isaac Sim from Launcher
```

### ROS 2 Workspace Setup

```bash
# Create workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws

# Clone Isaac ROS packages
cd src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common

# Build workspace
cd ~/isaac_ros_ws
colcon build
source install/setup.bash
```

## First Steps with Isaac Sim

### User Interface Overview

```
┌─────────────────────────────────────────────┐
│  Menu Bar                                    │
├─────┬───────────────────────────────┬───────┤
│     │                               │       │
│ S   │     Viewport                  │   P   │
│ t   │   (3D Scene)                  │   r   │
│ a   │                               │   o   │
│ g   │                               │   p   │
│ e   │                               │   e   │
│     │                               │   r   │
│ T   │                               │   t   │
│ r   │                               │   i   │
│ e   │                               │   e   │
│ e   │                               │   s   │
│     │                               │       │
├─────┴───────────────────────────────┴───────┤
│  Console / Python Scripts                   │
└─────────────────────────────────────────────┘
```

### Loading a Sample Scene

```python
# In Isaac Sim Python console
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

# Create world
world = World()

# Add a cube
cube = DynamicCuboid(
    prim_path="/World/Cube",
    position=[0, 0, 1],
    scale=[0.5, 0.5, 0.5],
    color=[1, 0, 0]
)

# Reset and play
world.reset()
simulation_app.update()
```

### Importing Robot Models

#### From URDF:

```python
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.importer.urdf")

from omni.importer.urdf import _urdf

# Import URDF
urdf_interface = _urdf.acquire_urdf_interface()
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = True

urdf_path = "/path/to/robot.urdf"
robot_prim_path = "/World/Robot"

urdf_interface.parse_urdf(urdf_path, robot_prim_path, import_config)
```

#### From USD:

```python
from pxr import Usd, UsdGeom
import omni.usd

# Load USD robot
stage = omni.usd.get_context().get_stage()
robot_prim = stage.DefinePrim("/World/Robot", "Xform")

# Reference external USD
robot_prim.GetReferences().AddReference("/path/to/robot.usd")
```

## ROS 2 Bridge

### Enabling ROS 2 Communication

```python
from omni.isaac.core.utils.extensions import enable_extension

# Enable ROS2 bridge
enable_extension("omni.isaac.ros2_bridge")

import omni.graph.core as og

# Create ROS2 graph
graph_path = "/World/ROS2_Graph"
(ros_graph, _, _, _) = og.Controller.edit(
    {"graph_path": graph_path, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
        ],
    },
)
```

### Publishing Camera Images

```python
# Create camera publisher graph
camera_graph_path = "/World/Camera_Graph"

(camera_graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": camera_graph_path, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("CameraHelper", "omni.isaac.core_nodes.IsaacReadCameraInfo"),
            ("ROS2CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("CameraHelper.inputs:cameraPrim", "/World/Camera"),
            ("ROS2CameraHelper.inputs:topicName", "rgb"),
            ("ROS2CameraHelper.inputs:frameId", "camera_link"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
            ("CameraHelper.outputs:execOut", "ROS2CameraHelper.inputs:execIn"),
            ("CameraHelper.outputs:cameraInfo", "ROS2CameraHelper.inputs:cameraInfo"),
        ],
    },
)
```

### Subscribing to Velocity Commands

```python
# Create cmd_vel subscriber
cmd_vel_graph_path = "/World/CmdVel_Graph"

(cmd_vel_graph, _, _, _) = og.Controller.edit(
    {"graph_path": cmd_vel_graph_path, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("SubscribeTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
            ("DifferentialController", "omni.isaac.wheeled_robots.DifferentialController"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("SubscribeTwist.inputs:topicName", "cmd_vel"),
            ("DifferentialController.inputs:wheelRadius", 0.1),
            ("DifferentialController.inputs:wheelDistance", 0.4),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
            ("SubscribeTwist.outputs:linearVelocity", "DifferentialController.inputs:linearVelocity"),
            ("SubscribeTwist.outputs:angularVelocity", "DifferentialController.inputs:angularVelocity"),
        ],
    },
)
```

## Synthetic Data Generation

Isaac Sim excels at generating labeled training data for AI models.

### Replicator API

```python
import omni.replicator.core as rep

# Create randomizer
def sphere_lights(num=10):
    lights = rep.create.light(
        light_type="Sphere",
        temperature=rep.distribution.uniform(3000, 6500),
        intensity=rep.distribution.uniform(20000, 40000),
        position=rep.distribution.uniform((-5, -5, 1), (5, 5, 5)),
        count=num
    )
    return lights.node

# Register randomizer
rep.randomizer.register(sphere_lights)

# Create camera
camera = rep.create.camera(position=(5, 5, 5), look_at=(0, 0, 0))

# Render
rp = rep.create.render_product(camera, (512, 512))

# Attach writer for annotations
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="~/output",
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=True
)
writer.attach([rp])

# Run randomization
with rep.trigger.on_frame(num_frames=100):
    rep.randomizer.sphere_lights(10)
```

### Data Collection

```python
# Collect diverse training data
import omni.replicator.core as rep

# Randomize object positions
def randomize_scene():
    cubes = rep.get.prims(path_pattern="/World/Cubes/*")

    with cubes:
        rep.modify.pose(
            position=rep.distribution.uniform((-2, -2, 0.5), (2, 2, 1.5)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
        )

    return cubes.node

rep.randomizer.register(randomize_scene)

# Collect 1000 samples
with rep.trigger.on_frame(num_frames=1000):
    rep.randomizer.randomize_scene()
```

## Key Takeaways

- **Isaac Platform** provides end-to-end robotics development tools
- **Isaac Sim** offers photorealistic simulation with GPU acceleration
- **ROS 2 Bridge** enables seamless integration with ROS ecosystem
- **Synthetic Data** generation accelerates AI model development
- **PhysX and RTX** provide accurate physics and rendering

## Practical Exercises

1. **Install Isaac Sim** and load sample scenes
2. **Import a URDF robot** and visualize in Isaac Sim
3. **Create ROS 2 bridge** for camera and velocity topics
4. **Generate synthetic dataset** with 100 annotated images

## Next Steps

In the next chapter, you'll learn about:
- Isaac navigation stack (Nav2 integration)
- Path planning and obstacle avoidance
- Sensor fusion and localization
- Autonomous navigation in warehouses
