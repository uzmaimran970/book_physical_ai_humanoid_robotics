---
id: module2-chapter3
title: Unity Simulation and Robotics
sidebar_label: Chapter 3 - Unity
---

# Unity Simulation and Robotics

## Introduction to Unity for Robotics

Unity is a professional game engine that has become increasingly popular for robotics simulation, particularly for computer vision, machine learning, and photorealistic rendering.

### Why Unity for Robotics?

- **Photorealistic Rendering**: High-quality visuals for computer vision training
- **ML-Agents**: Integrated reinforcement learning framework
- **Cross-Platform**: Windows, Linux, macOS support
- **Asset Ecosystem**: Massive library of 3D models and environments
- **Performance**: Optimized rendering and physics
- **VR/AR Support**: Human-robot interaction research

### Unity vs Traditional Robot Simulators

| Feature | Unity | Gazebo |
|---------|-------|--------|
| Rendering Quality | Excellent | Good |
| Physics | PhysX (Good) | Multiple engines (Better) |
| ML Integration | ML-Agents | External |
| ROS Integration | Via package | Native |
| Learning Curve | Steep | Moderate |
| Use Case | Vision/ML | General robotics |

## Unity Robotics Hub

The Unity Robotics Hub provides ROS integration and robotics-specific tools.

### Installation

#### 1. Install Unity Hub and Unity Editor

```bash
# Download Unity Hub
# https://unity.com/download

# Install Unity Editor (2021.3 LTS recommended)
# Select modules:
# - Linux Build Support
# - Documentation
```

#### 2. Install Unity Robotics Packages

In Unity Package Manager:
```
1. Window → Package Manager
2. Add package from git URL:
   - com.unity.robotics.ros-tcp-connector
   - com.unity.robotics.urdf-importer
   - com.unity.robotics.visualizations
```

#### 3. Install ROS-TCP-Endpoint (ROS 2 Side)

```bash
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Setting Up Unity-ROS 2 Communication

### Architecture

```
Unity Simulation  ←→  ROS-TCP-Connector  ←→  ROS-TCP-Endpoint  ←→  ROS 2 Nodes
    (C#)                  (Unity)             (Python/C++)         (Python/C++)
```

### Configuration

#### Unity Side (C#)

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class ROSConnection : MonoBehaviour
{
    void Start()
    {
        // Connect to ROS
        ROSConnection.GetOrCreateInstance().Connect();
    }
}
```

Create ROS Settings:
```
1. Robotics → ROS Settings
2. ROS IP Address: 192.168.1.100 (your ROS machine)
3. ROS Port: 10000
4. Protocol: ROS 2
```

#### ROS 2 Side

```bash
# Launch ROS-TCP-Endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args \
  -p ROS_IP:=0.0.0.0 \
  -p ROS_TCP_PORT:=10000
```

## Importing Robot Models

### Method 1: URDF Importer

```csharp
using Unity.Robotics.UrdfImporter;

public class RobotImporter : MonoBehaviour
{
    public string urdfFilePath = "Assets/URDF/my_robot.urdf";

    void Start()
    {
        // Import URDF
        UrdfRobot.Create(urdfFilePath);
    }
}
```

Workflow:
```
1. Assets → Import Robot from URDF
2. Select your URDF file
3. Configure import settings:
   - Axis convention: Z-Up (Unity) vs Z-Forward (ROS)
   - Mesh decomposition for collisions
   - Material assignments
4. Click Import
```

### Method 2: Manual CAD Import

```
1. Export from CAD (FBX, OBJ)
2. Import to Unity
3. Add components:
   - ArticulationBody (joints)
   - Colliders
   - ROS Publishers/Subscribers
```

## Publishing and Subscribing to ROS Topics

### Publishing from Unity

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class OdometryPublisher : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "/odom";
    public float publishRate = 20f;
    private float timer;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>(topicName);
    }

    void Update()
    {
        timer += Time.deltaTime;

        if (timer >= 1.0f / publishRate)
        {
            timer = 0;
            PublishOdometry();
        }
    }

    void PublishOdometry()
    {
        OdometryMsg odom = new OdometryMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                },
                frame_id = "odom"
            },
            child_frame_id = "base_link",
            pose = new PoseWithCovarianceMsg
            {
                pose = new PoseMsg
                {
                    position = new PointMsg
                    {
                        x = transform.position.x,
                        y = transform.position.z,  // Unity Y → ROS Z
                        z = transform.position.y   // Unity Z → ROS Y
                    },
                    orientation = new QuaternionMsg
                    {
                        x = transform.rotation.x,
                        y = transform.rotation.z,
                        z = transform.rotation.y,
                        w = transform.rotation.w
                    }
                }
            }
        };

        ros.Publish(topicName, odom);
    }
}
```

### Subscribing in Unity

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class VelocitySubscriber : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "/cmd_vel";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, ApplyVelocity);
    }

    void ApplyVelocity(TwistMsg msg)
    {
        // Convert ROS velocity to Unity movement
        float linearX = (float)msg.linear.x;
        float angularZ = (float)msg.angular.z;

        // Apply to robot (example using ArticulationBody)
        // Implementation depends on robot configuration
        Debug.Log($"Received velocity: linear={linearX}, angular={angularZ}");
    }
}
```

## Simulating Sensors

### RGB Camera

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

[RequireComponent(typeof(Camera))]
public class CameraPublisher : MonoBehaviour
{
    private Camera sensorCamera;
    private ROSConnection ros;
    private string topicName = "/camera/image_raw";
    public float publishRate = 30f;

    private RenderTexture renderTexture;
    private Texture2D texture2D;

    void Start()
    {
        sensorCamera = GetComponent<Camera>();
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        // Create render texture
        renderTexture = new RenderTexture(640, 480, 24);
        sensorCamera.targetTexture = renderTexture;
        texture2D = new Texture2D(640, 480, TextureFormat.RGB24, false);
    }

    void Update()
    {
        if (Time.frameCount % (int)(1.0f / publishRate * 60) == 0)
        {
            PublishImage();
        }
    }

    void PublishImage()
    {
        // Capture camera image
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, 640, 480), 0, 0);
        texture2D.Apply();

        // Convert to ROS message
        ImageMsg msg = new ImageMsg
        {
            header = new HeaderMsg
            {
                stamp = GetCurrentROSTime(),
                frame_id = "camera_link"
            },
            height = 480,
            width = 640,
            encoding = "rgb8",
            is_bigendian = 0,
            step = 640 * 3,
            data = texture2D.GetRawTextureData()
        };

        ros.Publish(topicName, msg);
    }

    TimeMsg GetCurrentROSTime()
    {
        double time = Time.time;
        return new TimeMsg
        {
            sec = (int)time,
            nanosec = (uint)((time % 1) * 1e9)
        };
    }
}
```

### 2D Lidar Scanner

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class LidarPublisher : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "/scan";

    public int numRays = 360;
    public float angleMin = -Mathf.PI;
    public float angleMax = Mathf.PI;
    public float rangeMin = 0.12f;
    public float rangeMax = 10.0f;
    public float scanRate = 10f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topicName);
        InvokeRepeating("PublishScan", 0, 1.0f / scanRate);
    }

    void PublishScan()
    {
        float[] ranges = new float[numRays];
        float angleIncrement = (angleMax - angleMin) / numRays;

        for (int i = 0; i < numRays; i++)
        {
            float angle = angleMin + i * angleIncrement;
            Vector3 direction = new Vector3(
                Mathf.Cos(angle),
                0,
                Mathf.Sin(angle)
            );

            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, rangeMax))
            {
                ranges[i] = hit.distance;
            }
            else
            {
                ranges[i] = float.PositiveInfinity;
            }
        }

        LaserScanMsg msg = new LaserScanMsg
        {
            header = new HeaderMsg
            {
                stamp = GetROSTime(),
                frame_id = "lidar_link"
            },
            angle_min = angleMin,
            angle_max = angleMax,
            angle_increment = angleIncrement,
            time_increment = 0,
            scan_time = 1.0f / scanRate,
            range_min = rangeMin,
            range_max = rangeMax,
            ranges = ranges
        };

        ros.Publish(topicName, msg);
    }
}
```

## Unity ML-Agents for Robotics

ML-Agents enables training reinforcement learning policies for robots.

### Installation

```bash
# Install ML-Agents package
# In Unity Package Manager, add:
com.unity.ml-agents

# Install Python package
pip install mlagents
```

### Creating a Training Environment

```csharp
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class RobotAgent : Agent
{
    public Transform target;
    private Rigidbody rBody;

    void Start()
    {
        rBody = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        // Reset robot position
        transform.localPosition = Vector3.zero;
        rBody.velocity = Vector3.zero;

        // Randomize target position
        target.localPosition = new Vector3(
            Random.Range(-4f, 4f),
            0.5f,
            Random.Range(-4f, 4f)
        );
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Robot position
        sensor.AddObservation(transform.localPosition);

        // Robot velocity
        sensor.AddObservation(rBody.velocity.x);
        sensor.AddObservation(rBody.velocity.z);

        // Target position
        sensor.AddObservation(target.localPosition);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Apply actions
        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = actions.ContinuousActions[0];
        controlSignal.z = actions.ContinuousActions[1];

        rBody.AddForce(controlSignal * 10f);

        // Calculate reward
        float distanceToTarget = Vector3.Distance(transform.localPosition, target.localPosition);

        // Reached target
        if (distanceToTarget < 1.5f)
        {
            SetReward(1.0f);
            EndEpisode();
        }

        // Fell off platform
        if (transform.localPosition.y < 0)
        {
            SetReward(-1.0f);
            EndEpisode();
        }

        // Small reward for getting closer
        AddReward(-0.001f);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Manual control for testing
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetAxis("Vertical");
    }
}
```

### Training Configuration

```yaml
# config.yaml
behaviors:
  RobotAgent:
    trainer_type: ppo
    hyperparameters:
      batch_size: 1024
      buffer_size: 10240
      learning_rate: 0.0003
      beta: 0.005
      epsilon: 0.2
      lambd: 0.95
      num_epoch: 3
    network_settings:
      normalize: false
      hidden_units: 128
      num_layers: 2
    reward_signals:
      extrinsic:
        gamma: 0.99
        strength: 1.0
    max_steps: 500000
    time_horizon: 64
    summary_freq: 10000
```

### Training Command

```bash
# Start training
mlagents-learn config.yaml --run-id=robot_navigation

# In Unity, press Play to start training
```

## Photorealistic Rendering for Computer Vision

### High Definition Render Pipeline (HDRP)

```
1. Create new HDRP project or upgrade existing:
   Window → Rendering → Render Pipeline Converter

2. Configure lighting:
   - Add HDRI Sky
   - Configure indirect lighting
   - Add reflection probes

3. Material setup:
   - Use HDRP/Lit shader
   - Add normal maps
   - Configure roughness/metallic
```

### Synthetic Data Generation

```csharp
using UnityEngine;
using UnityEngine.Perception.GroundTruth;

public class DatasetGenerator : MonoBehaviour
{
    public Camera perceptionCamera;
    public int framesToCapture = 1000;

    void Start()
    {
        // Add perception camera
        var perceptionCameraComponent = perceptionCamera.gameObject.AddComponent<PerceptionCamera>();

        // Add labeling
        perceptionCameraComponent.showVisualizations = true;

        // Capture bounding boxes
        var boundingBoxLabeler = new BoundingBox2DLabeler();
        perceptionCameraComponent.AddLabeler(boundingBoxLabeler);

        // Capture semantic segmentation
        var semanticLabeler = new SemanticSegmentationLabeler();
        perceptionCameraComponent.AddLabeler(semanticLabeler);
    }
}
```

## Module 2 Summary

### Key Concepts

#### Chapter 1: Digital Twins
- ✅ Digital twins enable safe, cost-effective robot development
- ✅ Simulation platforms vary in fidelity and use cases
- ✅ Sim-to-real gap requires careful mitigation
- ✅ URDF/SDF formats describe robot models

#### Chapter 2: Gazebo Simulation
- ✅ Gazebo provides accurate physics and ROS 2 integration
- ✅ Plugins add sensors and actuators
- ✅ World files define environments
- ✅ Launch files coordinate simulation startup

#### Chapter 3: Unity Simulation
- ✅ Unity excels at photorealistic rendering and ML
- ✅ ROS-TCP-Connector enables ROS 2 communication
- ✅ ML-Agents supports reinforcement learning
- ✅ Synthetic data generation for computer vision

### Practical Exercises

1. **Build Complete Robot Simulation**
   - URDF model with sensors
   - Gazebo world with obstacles
   - ROS 2 navigation stack
   - Unity alternative for vision tasks

2. **Train Navigation Policy**
   - Unity ML-Agents environment
   - Reward shaping for goal reaching
   - Deploy to ROS 2 robot
   - Compare sim and real performance

3. **Generate Synthetic Dataset**
   - 1000 labeled images
   - Bounding boxes for objects
   - Semantic segmentation
   - Train object detector

### Visual Diagrams (Conceptual)

```
Simulation Workflow:

Design Phase          Development Phase       Deployment Phase
    ↓                       ↓                       ↓
┌─────────┐          ┌──────────┐           ┌─────────────┐
│   CAD   │──────→   │ Gazebo/  │───────→   │ Real Robot  │
│ Design  │          │  Unity   │           │   Testing   │
└─────────┘          └──────────┘           └─────────────┘
                          ↓
                    ┌──────────┐
                    │ROS 2 Code│
                    └──────────┘
```

### Resources for Further Learning

- **Unity Robotics Hub**: github.com/Unity-Technologies/Unity-Robotics-Hub
- **Gazebo Tutorials**: gazebosim.org/tutorials
- **ML-Agents**: github.com/Unity-Technologies/ml-agents
- **ROS 2 + Gazebo**: docs.ros.org/en/humble/Tutorials/Gazebo

### Next Module Preview

**Module 3: The AI-Robot Brain (NVIDIA Isaac™)**
- NVIDIA Isaac Sim overview
- Navigation stack (Nav2)
- Perception and manipulation
- AI-accelerated robotics workflows
