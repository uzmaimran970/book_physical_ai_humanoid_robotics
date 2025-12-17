#!/usr/bin/env python3
"""
Add comprehensive robotics content including basic definitions
"""
import sys
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.services.embeddings import EmbeddingService
from src.services.vector_store import VectorStoreService

# Comprehensive robotics content with basic definitions
COMPREHENSIVE_CHUNKS = [
    {
        "text": "Robotics is the interdisciplinary field that combines engineering, computer science, and artificial intelligence to design, construct, and operate robots. Robots are programmable machines capable of carrying out complex series of actions automatically. The field encompasses mechanical engineering (for physical structure), electrical engineering (for sensors and actuators), and computer science (for control algorithms and AI). Key areas include manipulation, locomotion, perception, planning, and human-robot interaction.",
        "metadata": {"chapter": "1", "section": "1.0", "page": "5", "title": "What is Robotics"}
    },
    {
        "text": "A robot is an autonomous or semi-autonomous machine that can sense its environment, make decisions, and take actions to accomplish tasks. Robots typically consist of: (1) Mechanical structure (body, arms, wheels, legs), (2) Actuators (motors that enable movement), (3) Sensors (cameras, LIDAR, touch sensors for perception), (4) Controller (computer that processes information and makes decisions), and (5) Power source (batteries or external power). Robots can be mobile (moving around) or fixed (like industrial robot arms).",
        "metadata": {"chapter": "1", "section": "1.0", "page": "6", "title": "Definition of a Robot"}
    },
    {
        "text": "Services and Actions are key concepts in robot programming frameworks like ROS (Robot Operating System). A Service is a synchronous request-response communication pattern where a client sends a request and waits for a response (e.g., 'turn on camera'). An Action is an asynchronous communication pattern for long-running tasks that provides feedback and can be cancelled (e.g., 'navigate to location X'). Services are used for quick computations, while Actions are used for tasks that take time to complete.",
        "metadata": {"chapter": "8", "section": "8.2", "page": "289", "title": "ROS Services and Actions"}
    },
    {
        "text": "Robot control systems manage how robots move and interact with their environment. Open-loop control executes pre-programmed commands without feedback. Closed-loop control (feedback control) uses sensors to monitor the robot's state and adjust actions accordingly. Common control approaches include: PID control (Proportional-Integral-Derivative) for maintaining desired positions, Model Predictive Control (MPC) for planning ahead, and Adaptive control for changing environments. Modern robots often use learning-based control with neural networks.",
        "metadata": {"chapter": "5", "section": "5.1", "page": "145", "title": "Robot Control Systems"}
    },
    {
        "text": "Robot perception enables robots to understand their environment through sensors. Vision systems use cameras (RGB, depth, infrared) to detect objects, people, and obstacles. LIDAR (Light Detection and Ranging) measures distances using laser beams, creating 3D point clouds. Tactile sensors provide touch feedback. IMUs (Inertial Measurement Units) measure acceleration and rotation. Perception algorithms process this sensor data to build representations of the environment, detect objects, and track movement.",
        "metadata": {"chapter": "4", "section": "4.0", "page": "105", "title": "Robot Perception"}
    },
    {
        "text": "Path planning is the process of finding a collision-free path from a start location to a goal location. Common algorithms include: A* (optimal path finding in known environments), RRT (Rapidly-exploring Random Trees for complex spaces), Dijkstra's algorithm (shortest path), and Potential Fields (treat goal as attractive force, obstacles as repulsive). Dynamic path planning adapts to moving obstacles. Motion planning extends this by considering robot kinematics and dynamics constraints.",
        "metadata": {"chapter": "5", "section": "5.4", "page": "178", "title": "Path Planning"}
    },
    {
        "text": "Robot kinematics studies the geometry of motion without considering forces. Forward kinematics calculates the end-effector position given joint angles (e.g., where is the robot hand?). Inverse kinematics solves for joint angles needed to reach a desired position (e.g., how should joints move to reach point X?). This is crucial for robot arms and manipulators. Differential kinematics relates joint velocities to end-effector velocities, used in velocity control and trajectory planning.",
        "metadata": {"chapter": "3", "section": "3.3", "page": "95", "title": "Robot Kinematics"}
    },
    {
        "text": "Robot dynamics studies forces and torques that cause motion. It considers mass, inertia, friction, and gravity. The dynamic model predicts how forces produce motion (forward dynamics) or calculates forces needed for desired motion (inverse dynamics). Key equations include the Lagrangian formulation and Newton-Euler method. Dynamics is essential for force control, trajectory tracking, and simulation. Modern approaches use learning to model complex dynamics that are hard to model analytically.",
        "metadata": {"chapter": "3", "section": "3.4", "page": "98", "title": "Robot Dynamics"}
    },
    {
        "text": "Robot operating systems (ROS) provide software frameworks for robot development. ROS offers: message passing for inter-process communication, hardware abstraction, device drivers, libraries for common robotics tasks (navigation, manipulation), visualization tools (RViz), and simulation (Gazebo). Nodes are independent processes that communicate via topics (pub-sub), services (request-response), and actions (long-running tasks). ROS 2 improves on ROS 1 with better real-time support and security.",
        "metadata": {"chapter": "8", "section": "8.1", "page": "275", "title": "Robot Operating Systems"}
    },
    {
        "text": "Degrees of freedom (DOF) in robotics refers to the number of independent ways a robot can move. A robot arm with 6 DOF can position and orient its end-effector anywhere in 3D space (3 position + 3 orientation). More DOF provides greater dexterity but increases control complexity. Redundant robots have more DOF than needed for a task, allowing obstacle avoidance while reaching goals. Humanoid robots can have 30+ DOF to mimic human movement.",
        "metadata": {"chapter": "2", "section": "2.2", "page": "52", "title": "Degrees of Freedom"}
    },
]

def add_comprehensive_data():
    """Add comprehensive robotics content to Qdrant"""
    print("🚀 Adding comprehensive robotics content...")

    embeddings_service = EmbeddingService()
    vector_store = VectorStoreService()

    vector_store.ensure_collection()
    print("✅ Collection ready")

    texts = [chunk["text"] for chunk in COMPREHENSIVE_CHUNKS]

    print(f"🔄 Generating embeddings for {len(texts)} chunks...")
    embeddings = embeddings_service.embed_chunks_sync(texts)
    print(f"✅ Generated {len(embeddings)} embeddings")

    chunks_to_upsert = []
    for i, (chunk, embedding) in enumerate(zip(COMPREHENSIVE_CHUNKS, embeddings), 1):
        metadata = chunk["metadata"]
        chunk_data = {
            "chunk_id": str(uuid.uuid4()),
            "embedding": embedding,
            "text": chunk["text"],
            "chapter": metadata["chapter"],
            "section": metadata["section"],
            "page": metadata["page"],
            "book_id": "comprehensive_robotics",
            "chunk_index": i,
            "token_count": len(chunk["text"].split())
        }
        chunks_to_upsert.append(chunk_data)

    print("💾 Storing in Qdrant...")
    vector_store.upsert_chunks(chunks_to_upsert)
    print(f"✅ Added {len(chunks_to_upsert)} comprehensive chunks!")

    print("\n🎉 Now you can ask:")
    print("   - What is robotics?")
    print("   - What is a robot?")
    print("   - What are services and actions?")
    print("   - Explain robot control systems")
    print("   - What is path planning?")
    print("   - What are degrees of freedom?")

if __name__ == "__main__":
    add_comprehensive_data()
