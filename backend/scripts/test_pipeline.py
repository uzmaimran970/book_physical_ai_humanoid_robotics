"""Test RAG pipeline with sample queries.

Validates that the complete RAG pipeline works end-to-end.
"""
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.models.entities import QueryRequest
from src.services.rag_pipeline import rag_pipeline
from src.services.vector_store import vector_store_service


def test_health_check():
    """Test that services are accessible."""
    print("Testing service connectivity...")

    try:
        # Test Qdrant connection
        collections = vector_store_service.client.get_collections()
        print(f"✓ Qdrant connected ({len(collections.collections)} collections)")

        # Check if our collection exists
        collection_names = [c.name for c in collections.collections]
        if vector_store_service.collection_name in collection_names:
            print(f"✓ Collection '{vector_store_service.collection_name}' exists")
        else:
            print(f"⚠ Collection '{vector_store_service.collection_name}' not found")
            print("  Run book ingestion script first!")
            return False

        return True

    except Exception as e:
        print(f"✗ Service check failed: {e}")
        return False


def test_query(query_text: str, top_k: int = 3):
    """Test a single query.

    Args:
        query_text: Question to ask
        top_k: Number of chunks to retrieve
    """
    print(f"\n{'='*60}")
    print(f"Query: {query_text}")
    print(f"{'='*60}\n")

    try:
        # Create request
        request = QueryRequest(query=query_text, top_k=top_k)

        # Execute pipeline
        response = rag_pipeline.query(request)

        # Display results
        print(f"Status: {response.status}")
        print(f"Latency: {response.latency_ms}ms")
        print(f"Query ID: {response.query_id}\n")

        print("Answer:")
        print("-" * 60)
        print(response.answer)
        print("-" * 60)

        if response.citations:
            print(f"\nCitations ({len(response.citations)}):")
            for i, citation in enumerate(response.citations, 1):
                parts = []
                if citation.chapter:
                    parts.append(f"Chapter {citation.chapter}")
                if citation.section:
                    parts.append(citation.section)
                if citation.page:
                    parts.append(f"p. {citation.page}")

                citation_str = ", ".join(parts) if parts else "Unknown source"
                print(f"  [{i}] {citation_str}")
                print(f"      Relevance: {citation.relevance_score:.3f}")
                print(f"      Chunk ID: {citation.chunk_id}")

        return True

    except Exception as e:
        print(f"✗ Query failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run test queries."""
    print("RAG Pipeline Test\n")

    # Check services
    if not test_health_check():
        print("\nService health check failed. Please fix configuration and try again.")
        sys.exit(1)

    # Sample queries
    queries = [
        "What are IMU sensors?",
        "Explain the role of sensors in humanoid robotics",
        "What is physical AI?",
    ]

    # Run tests
    success_count = 0
    for query in queries:
        if test_query(query):
            success_count += 1

    # Summary
    print(f"\n{'='*60}")
    print(f"Test Summary: {success_count}/{len(queries)} queries successful")
    print(f"{'='*60}")

    if success_count == len(queries):
        print("\n✓ All tests passed!")
        sys.exit(0)
    else:
        print(f"\n⚠ {len(queries) - success_count} test(s) failed")
        sys.exit(1)


if __name__ == "__main__":
    main()
