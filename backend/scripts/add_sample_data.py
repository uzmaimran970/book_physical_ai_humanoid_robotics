#!/usr/bin/env python3
"""
Add sample Physical AI & Humanoid Robotics textbook content to Qdrant
"""
import sys
import uuid
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.services.embeddings import EmbeddingService
from src.services.vector_store import VectorStoreService
from src.utils.config import config

# Sample textbook chunks about Physical AI and Humanoid Robotics
SAMPLE_CHUNKS = [
    {
        "text": "Physical AI refers to artificial intelligence systems that interact with and manipulate the physical world. Unlike traditional AI that exists purely in the digital realm, Physical AI combines perception, reasoning, and actuation to enable robots and autonomous systems to understand and navigate real-world environments.",
        "metadata": {
            "chapter": "1",
            "section": "1.1",
            "page": "12",
            "title": "Introduction to Physical AI"
        }
    },
    {
        "text": "Humanoid robots are designed to resemble the human body in shape and movement. They typically feature a torso, head, two arms, and two legs. The humanoid form factor provides several advantages: intuitive human-robot interaction, ability to use human tools and environments, and easier acceptance in social contexts.",
        "metadata": {
            "chapter": "2",
            "section": "2.1",
            "page": "45",
            "title": "Humanoid Robot Design"
        }
    },
    {
        "text": "Embodied AI emphasizes the importance of physical form in intelligence. The hypothesis states that intelligence emerges from the interaction between an agent's body, brain, and environment. This contrasts with disembodied AI approaches that treat intelligence as pure computation.",
        "metadata": {
            "chapter": "1",
            "section": "1.3",
            "page": "28",
            "title": "Embodied Intelligence"
        }
    },
    {
        "text": "Locomotion in humanoid robots involves complex control systems. Bipedal walking requires precise balance control, gait planning, and real-time adjustment to terrain variations. Common approaches include Zero Moment Point (ZMP) control, inverse kinematics, and learning-based methods using reinforcement learning.",
        "metadata": {
            "chapter": "3",
            "section": "3.2",
            "page": "87",
            "title": "Bipedal Locomotion"
        }
    },
    {
        "text": "Sensor fusion combines data from multiple sensors to create a coherent understanding of the environment. Humanoid robots typically integrate cameras (RGB and depth), IMUs (Inertial Measurement Units), force/torque sensors, and tactile sensors. Kalman filters and particle filters are commonly used for sensor fusion.",
        "metadata": {
            "chapter": "4",
            "section": "4.1",
            "page": "112",
            "title": "Perception and Sensor Fusion"
        }
    },
    {
        "text": "Manipulation with humanoid hands requires dexterous control. The human hand has 27 degrees of freedom, enabling complex grasping and manipulation. Robot hands use various actuation methods: tendon-driven systems, direct motor drives, or soft robotics approaches. Grasp planning algorithms determine optimal finger positions for different objects.",
        "metadata": {
            "chapter": "5",
            "section": "5.3",
            "page": "156",
            "title": "Dexterous Manipulation"
        }
    },
    {
        "text": "Reinforcement learning has revolutionized robot control. Instead of hand-coding behaviors, robots learn through trial and error. Policy gradient methods like PPO (Proximal Policy Optimization) and SAC (Soft Actor-Critic) are popular for continuous control tasks. Simulation environments like MuJoCo and Isaac Gym enable rapid learning.",
        "metadata": {
            "chapter": "6",
            "section": "6.2",
            "page": "203",
            "title": "Learning-Based Control"
        }
    },
    {
        "text": "Human-robot interaction (HRI) involves communication between humans and robots. This includes verbal communication (speech recognition and synthesis), non-verbal cues (gestures, facial expressions), and haptic feedback. Social robots must understand context, emotions, and cultural norms to interact naturally.",
        "metadata": {
            "chapter": "7",
            "section": "7.1",
            "page": "241",
            "title": "Human-Robot Interaction"
        }
    },
    {
        "text": "SLAM (Simultaneous Localization and Mapping) enables robots to build maps while tracking their position. Visual SLAM uses camera data, while LiDAR SLAM provides accurate 3D mapping. Modern approaches combine multiple sensors and use graph optimization techniques like pose graph optimization.",
        "metadata": {
            "chapter": "4",
            "section": "4.3",
            "page": "134",
            "title": "Navigation and SLAM"
        }
    },
    {
        "text": "Actuators convert electrical signals into mechanical motion. Humanoid robots use various actuator types: electric motors (DC, brushless DC, servo), hydraulic actuators for high force applications, and pneumatic actuators for compliant motion. Series elastic actuators (SEAs) provide force control and shock absorption.",
        "metadata": {
            "chapter": "3",
            "section": "3.1",
            "page": "73",
            "title": "Actuators and Power Systems"
        }
    }
]

def add_sample_data():
    """Add sample textbook chunks to Qdrant"""
    print("🚀 Initializing services...")

    # Initialize services
    embeddings_service = EmbeddingService()
    vector_store = VectorStoreService()

    # Ensure collection exists
    vector_store.ensure_collection()
    print("✅ Collection ready")

    print(f"\n📚 Adding {len(SAMPLE_CHUNKS)} sample chunks...")

    # Collect all texts for batch embedding
    texts = [chunk["text"] for chunk in SAMPLE_CHUNKS]

    # Generate embeddings in batch
    print("🔄 Generating embeddings...")
    embeddings = embeddings_service.embed_chunks_sync(texts)
    print(f"✅ Generated {len(embeddings)} embeddings")

    # Prepare chunks for upsert
    chunks_to_upsert = []
    for i, (chunk, embedding) in enumerate(zip(SAMPLE_CHUNKS, embeddings), 1):
        metadata = chunk["metadata"]
        chunk_data = {
            "chunk_id": str(uuid.uuid4()),
            "embedding": embedding,
            "text": chunk["text"],
            "chapter": metadata["chapter"],
            "section": metadata["section"],
            "page": metadata["page"],
            "book_id": "sample_book",
            "chunk_index": i,
            "token_count": len(chunk["text"].split())
        }
        chunks_to_upsert.append(chunk_data)

    # Upsert all chunks
    print("💾 Storing in Qdrant...")
    vector_store.upsert_chunks(chunks_to_upsert)
    print(f"✅ Stored {len(chunks_to_upsert)} chunks")

    print(f"\n🎉 Successfully added {len(SAMPLE_CHUNKS)} chunks to Qdrant!")
    print("\n📊 Collection stats:")
    collection_info = vector_store.client.get_collection(config.QDRANT_COLLECTION)
    print(f"   Total points: {collection_info.points_count}")
    print(f"   Vector size: {collection_info.config.params.vectors.size}")

    print("\n✅ Sample data loaded! You can now ask questions about:")
    print("   - Physical AI and embodied intelligence")
    print("   - Humanoid robot design and locomotion")
    print("   - Sensor fusion and perception")
    print("   - Manipulation and grasping")
    print("   - Reinforcement learning for robotics")
    print("   - Human-robot interaction")
    print("   - SLAM and navigation")

if __name__ == "__main__":
    add_sample_data()
