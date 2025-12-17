"""Validate environment setup.

Checks that all required configuration and services are properly set up.
"""
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))


def check_imports():
    """Verify all required packages can be imported."""
    print("Checking package imports...")
    required_packages = [
        ("fastapi", "FastAPI"),
        ("uvicorn", "Uvicorn"),
        ("pydantic", "Pydantic"),
        ("cohere", "Cohere"),
        ("qdrant_client", "Qdrant Client"),
        ("anthropic", "Anthropic"),
        ("asyncpg", "asyncpg"),
        ("tiktoken", "tiktoken"),
        ("dotenv", "python-dotenv"),
    ]

    all_ok = True
    for package, name in required_packages:
        try:
            __import__(package)
            print(f"  ✓ {name}")
        except ImportError:
            print(f"  ✗ {name} - not installed")
            all_ok = False

    return all_ok


def check_env_vars():
    """Verify all required environment variables are set."""
    print("\nChecking environment variables...")

    try:
        from src.utils.config import config

        required_vars = [
            ("QDRANT_URL", config.QDRANT_URL, "Qdrant cluster URL"),
            ("QDRANT_API_KEY", config.QDRANT_API_KEY, "Qdrant API key"),
            ("COHERE_API_KEY", config.COHERE_API_KEY, "Cohere API key"),
            ("ANTHROPIC_API_KEY", config.ANTHROPIC_API_KEY, "Anthropic API key"),
            ("NEON_DB_URL", config.NEON_DB_URL, "Neon database URL"),
        ]

        all_ok = True
        for var_name, var_value, description in required_vars:
            if var_value:
                # Mask the value for security
                masked = var_value[:8] + "..." if len(var_value) > 8 else "***"
                print(f"  ✓ {var_name}: {masked}")
            else:
                print(f"  ✗ {var_name}: Not set ({description})")
                all_ok = False

        return all_ok

    except Exception as e:
        print(f"  ✗ Error loading config: {e}")
        return False


def check_services():
    """Test connectivity to external services."""
    print("\nChecking service connectivity...")

    all_ok = True

    # Test Qdrant
    try:
        from src.services.vector_store import vector_store_service

        collections = vector_store_service.client.get_collections()
        print(f"  ✓ Qdrant: Connected ({len(collections.collections)} collections)")

        # Check for our collection
        collection_names = [c.name for c in collections.collections]
        if vector_store_service.collection_name in collection_names:
            print(f"    ✓ Collection '{vector_store_service.collection_name}' exists")
        else:
            print(f"    ⚠ Collection '{vector_store_service.collection_name}' not found")
            print("      (Run book ingestion script to create it)")

    except Exception as e:
        print(f"  ✗ Qdrant: Connection failed - {e}")
        all_ok = False

    # Test Cohere (basic check)
    try:
        from src.services.embeddings import embedding_service

        # Try a simple embedding
        test_vector = embedding_service.embed_query("test")
        if len(test_vector) == 1024:
            print(f"  ✓ Cohere: Connected (embedding dimension: {len(test_vector)})")
        else:
            print(f"  ⚠ Cohere: Unexpected embedding dimension: {len(test_vector)}")

    except Exception as e:
        print(f"  ✗ Cohere: Connection failed - {e}")
        all_ok = False

    # Test Claude (basic check)
    try:
        from src.services.llm import llm_service

        # Just verify we have the client configured
        if llm_service.client and llm_service.model:
            print(f"  ✓ Claude: Configured (model: {llm_service.model})")
        else:
            print(f"  ✗ Claude: Not properly configured")
            all_ok = False

    except Exception as e:
        print(f"  ✗ Claude: Setup failed - {e}")
        all_ok = False

    return all_ok


def check_file_structure():
    """Verify project file structure."""
    print("\nChecking file structure...")

    base_path = Path(__file__).parent.parent
    required_paths = [
        "src/__init__.py",
        "src/main.py",
        "src/api/routes.py",
        "src/models/entities.py",
        "src/services/embeddings.py",
        "src/services/vector_store.py",
        "src/services/llm.py",
        "src/services/rag_pipeline.py",
        "src/utils/config.py",
        "src/utils/error_handling.py",
        "requirements.txt",
        ".env",
    ]

    all_ok = True
    for rel_path in required_paths:
        full_path = base_path / rel_path
        if full_path.exists():
            print(f"  ✓ {rel_path}")
        else:
            print(f"  ✗ {rel_path} - not found")
            all_ok = False

    return all_ok


def main():
    """Run all validation checks."""
    print("RAG Chatbot Setup Validation\n")
    print("=" * 60)

    checks = [
        ("Package imports", check_imports),
        ("Environment variables", check_env_vars),
        ("File structure", check_file_structure),
        ("Service connectivity", check_services),
    ]

    results = []
    for check_name, check_func in checks:
        result = check_func()
        results.append((check_name, result))

    # Summary
    print("\n" + "=" * 60)
    print("Validation Summary\n")

    all_passed = True
    for check_name, result in results:
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"  {status}: {check_name}")
        if not result:
            all_passed = False

    print("=" * 60)

    if all_passed:
        print("\n✓ All validation checks passed!")
        print("\nNext steps:")
        print("  1. Ingest a textbook: python scripts/ingest_book.py --text-file <path>")
        print("  2. Test pipeline: python scripts/test_pipeline.py")
        print("  3. Start API: python -m src.main")
        sys.exit(0)
    else:
        print("\n⚠ Some validation checks failed.")
        print("\nPlease fix the issues above and run this script again.")
        sys.exit(1)


if __name__ == "__main__":
    main()
