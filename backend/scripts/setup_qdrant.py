"""Setup Qdrant collection script.

Creates the textbook_chunks collection in Qdrant with proper configuration.
"""
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.services.vector_store import vector_store_service
from src.utils.config import config


def setup_qdrant_collection():
    """Setup Qdrant collection."""
    print("Setting up Qdrant collection...")
    print(f"Qdrant URL: {config.QDRANT_URL}")
    print(f"Collection name: {vector_store_service.collection_name}")

    try:
        # Create collection
        vector_store_service.ensure_collection()
        print(f"\n✓ Collection '{vector_store_service.collection_name}' created successfully!")

        # Verify collection exists
        collections = vector_store_service.client.get_collections()
        collection_names = [c.name for c in collections.collections]

        if vector_store_service.collection_name in collection_names:
            print(f"✓ Verified: Collection exists in Qdrant")

            # Get collection info
            collection_info = vector_store_service.client.get_collection(
                collection_name=vector_store_service.collection_name
            )
            print(f"\nCollection details:")
            print(f"  - Vectors: {collection_info.points_count}")
            print(f"  - Vector size: {vector_store_service.vector_size}")
            print(f"  - Distance: Cosine")
        else:
            print("⚠ Warning: Collection not found after creation")

    except Exception as e:
        print(f"\n✗ Failed to setup collection: {e}")
        sys.exit(1)


if __name__ == "__main__":
    setup_qdrant_collection()
