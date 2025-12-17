"""Database service for query logging.

Provides async database operations for logging queries, responses, and analytics.
"""
import asyncpg
from typing import List, Dict, Any, Optional
from uuid import UUID
from datetime import datetime
from contextlib import asynccontextmanager
from src.utils.config import config
from src.utils.error_handling import DatabaseError


class DatabaseService:
    """Async database service for Neon Postgres."""

    def __init__(self):
        """Initialize database service."""
        self.pool: Optional[asyncpg.Pool] = None
        self.db_url = config.NEON_DB_URL

    async def initialize(self) -> None:
        """Create connection pool.

        Raises:
            DatabaseError: If pool creation fails
        """
        try:
            self.pool = await asyncpg.create_pool(
                self.db_url,
                min_size=2,
                max_size=10,
                command_timeout=60
            )
        except Exception as e:
            raise DatabaseError(f"Failed to create connection pool: {str(e)}") from e

    async def close(self) -> None:
        """Close connection pool."""
        if self.pool:
            await self.pool.close()

    @asynccontextmanager
    async def connection(self):
        """Get a connection from the pool.

        Yields:
            Connection from pool

        Raises:
            DatabaseError: If connection acquisition fails
        """
        if not self.pool:
            raise DatabaseError("Database pool not initialized")

        conn = await self.pool.acquire()
        try:
            yield conn
        finally:
            await self.pool.release(conn)

    async def log_query(
        self,
        query_id: UUID,
        query_text: str,
        top_k: int,
        timestamp: datetime
    ) -> None:
        """Log a query to the database.

        Args:
            query_id: Unique query identifier
            query_text: User's question
            top_k: Number of chunks retrieved
            timestamp: Query timestamp

        Raises:
            DatabaseError: If logging fails
        """
        try:
            async with self.connection() as conn:
                await conn.execute(
                    """
                    INSERT INTO queries (query_id, query_text, top_k, timestamp)
                    VALUES ($1, $2, $3, $4)
                    """,
                    query_id,
                    query_text,
                    top_k,
                    timestamp
                )
        except Exception as e:
            raise DatabaseError(f"Failed to log query: {str(e)}") from e

    async def log_response(
        self,
        response_id: UUID,
        query_id: UUID,
        response_text: str,
        citations: List[Dict[str, Any]],
        latency_ms: int,
        status: str
    ) -> None:
        """Log a response to the database.

        Args:
            response_id: Unique response identifier
            query_id: Associated query ID
            response_text: Generated answer
            citations: List of citation dicts
            latency_ms: Processing time in milliseconds
            status: Query outcome

        Raises:
            DatabaseError: If logging fails
        """
        try:
            async with self.connection() as conn:
                await conn.execute(
                    """
                    INSERT INTO responses (
                        response_id, query_id, response_text,
                        citations, latency_ms, status
                    )
                    VALUES ($1, $2, $3, $4, $5, $6)
                    """,
                    response_id,
                    query_id,
                    response_text,
                    citations,  # JSONB
                    latency_ms,
                    status
                )
        except Exception as e:
            raise DatabaseError(f"Failed to log response: {str(e)}") from e

    async def log_retrieved_chunks(
        self,
        query_id: UUID,
        chunks: List[Dict[str, Any]]
    ) -> None:
        """Log retrieved chunks for a query.

        Args:
            query_id: Associated query ID
            chunks: List of chunk dicts with chunk_id, score, rank

        Raises:
            DatabaseError: If logging fails
        """
        try:
            async with self.connection() as conn:
                # Insert multiple rows
                await conn.executemany(
                    """
                    INSERT INTO retrieved_chunks (
                        query_id, chunk_id, relevance_score, rank
                    )
                    VALUES ($1, $2, $3, $4)
                    """,
                    [
                        (query_id, chunk["chunk_id"], chunk["score"], chunk["rank"])
                        for chunk in chunks
                    ]
                )
        except Exception as e:
            raise DatabaseError(f"Failed to log retrieved chunks: {str(e)}") from e

    async def log_error(
        self,
        error_id: UUID,
        query_id: Optional[UUID],
        error_code: str,
        error_message: str,
        timestamp: datetime
    ) -> None:
        """Log an error to the database.

        Args:
            error_id: Unique error identifier
            query_id: Associated query ID (if applicable)
            error_code: Machine-readable error code
            error_message: Human-readable error message
            timestamp: Error timestamp

        Raises:
            DatabaseError: If logging fails
        """
        try:
            async with self.connection() as conn:
                await conn.execute(
                    """
                    INSERT INTO error_logs (
                        error_id, query_id, error_code,
                        error_message, timestamp
                    )
                    VALUES ($1, $2, $3, $4, $5)
                    """,
                    error_id,
                    query_id,
                    error_code,
                    error_message,
                    timestamp
                )
        except Exception as e:
            # Don't raise here to avoid error logging loop
            print(f"Failed to log error: {e}")

    async def get_query_stats(
        self,
        start_date: Optional[datetime] = None,
        end_date: Optional[datetime] = None
    ) -> Dict[str, Any]:
        """Get query statistics.

        Args:
            start_date: Start of date range (optional)
            end_date: End of date range (optional)

        Returns:
            Dict with total_queries, avg_latency_ms, success_rate

        Raises:
            DatabaseError: If query fails
        """
        try:
            async with self.connection() as conn:
                query = """
                    SELECT
                        COUNT(*) as total_queries,
                        AVG(r.latency_ms) as avg_latency_ms,
                        SUM(CASE WHEN r.status = 'success' THEN 1 ELSE 0 END)::float /
                            COUNT(*) as success_rate
                    FROM queries q
                    LEFT JOIN responses r ON q.query_id = r.query_id
                    WHERE 1=1
                """

                params = []
                if start_date:
                    params.append(start_date)
                    query += f" AND q.timestamp >= ${len(params)}"

                if end_date:
                    params.append(end_date)
                    query += f" AND q.timestamp <= ${len(params)}"

                row = await conn.fetchrow(query, *params)

                return {
                    "total_queries": row["total_queries"] or 0,
                    "avg_latency_ms": float(row["avg_latency_ms"] or 0),
                    "success_rate": float(row["success_rate"] or 0)
                }

        except Exception as e:
            raise DatabaseError(f"Failed to get query stats: {str(e)}") from e

    async def get_top_errors(self, limit: int = 10) -> List[Dict[str, Any]]:
        """Get most common errors.

        Args:
            limit: Number of errors to return

        Returns:
            List of dicts with error_code, count

        Raises:
            DatabaseError: If query fails
        """
        try:
            async with self.connection() as conn:
                rows = await conn.fetch(
                    """
                    SELECT error_code, COUNT(*) as count
                    FROM error_logs
                    GROUP BY error_code
                    ORDER BY count DESC
                    LIMIT $1
                    """,
                    limit
                )

                return [
                    {"error_code": row["error_code"], "count": row["count"]}
                    for row in rows
                ]

        except Exception as e:
            raise DatabaseError(f"Failed to get top errors: {str(e)}") from e


# Singleton instance
database_service = DatabaseService()
