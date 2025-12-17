"""Setup database schema script.

Executes the SQL schema file against the Neon database.
"""
import asyncio
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.services.database import database_service
from src.utils.config import config


async def setup_database():
    """Setup database schema."""
    print("Setting up database schema...")
    print(f"Database: {config.NEON_DB_URL.split('@')[1] if '@' in config.NEON_DB_URL else 'unknown'}")

    # Read SQL schema file
    sql_file = Path(__file__).parent / "setup_database.sql"
    if not sql_file.exists():
        print(f"Error: SQL file not found: {sql_file}")
        sys.exit(1)

    with open(sql_file, "r", encoding="utf-8") as f:
        sql_schema = f.read()

    # Initialize database connection
    try:
        await database_service.initialize()
        print("✓ Connected to database")
    except Exception as e:
        print(f"✗ Failed to connect to database: {e}")
        sys.exit(1)

    # Execute schema
    try:
        async with database_service.connection() as conn:
            # Split by semicolons and execute each statement
            statements = [s.strip() for s in sql_schema.split(";") if s.strip()]

            for idx, statement in enumerate(statements, 1):
                # Skip comments and empty lines
                if statement.startswith("--") or not statement:
                    continue

                try:
                    await conn.execute(statement)
                    print(f"  ✓ Executed statement {idx}/{len(statements)}")
                except Exception as e:
                    # Some statements might fail if objects already exist
                    if "already exists" in str(e).lower():
                        print(f"  ⚠ Statement {idx} already exists: {str(e)[:50]}...")
                    else:
                        print(f"  ✗ Statement {idx} failed: {e}")

        print("\n✓ Database schema setup complete!")

        # Verify tables
        async with database_service.connection() as conn:
            tables = await conn.fetch("""
                SELECT tablename
                FROM pg_tables
                WHERE schemaname = 'public'
                ORDER BY tablename
            """)

            print(f"\nCreated {len(tables)} tables:")
            for table in tables:
                print(f"  - {table['tablename']}")

    except Exception as e:
        print(f"\n✗ Schema setup failed: {e}")
        sys.exit(1)
    finally:
        await database_service.close()


def main():
    """CLI entry point."""
    try:
        asyncio.run(setup_database())
    except KeyboardInterrupt:
        print("\n\nSetup cancelled by user")
        sys.exit(1)


if __name__ == "__main__":
    main()
