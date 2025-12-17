"""Qdrant vector store service.

Provides vector search and storage functionality using Qdrant Cloud.
"""
from typing import List, Dict, Any
from uuid import UUID
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, SearchRequest
from src.utils.config import config
from src.utils.error_handling import QdrantError


class VectorStoreService:
    """Wrapper for Qdrant vector database."""

    def __init__(self):
        """Initialize Qdrant client."""
        self.client = QdrantClient(
            url=config.QDRANT_URL,
            api_key=config.QDRANT_API_KEY
        )
        self.collection_name = config.QDRANT_COLLECTION
        self.vector_size = 1024  # Cohere embed-english-v3.0

    def ensure_collection(self) -> None:
        """Create collection if it doesn't exist.

        Raises:
            QdrantError: If collection creation fails
        """
        try:
            collections = self.client.get_collections().collections
            collection_names = [c.name for c in collections]

            if self.collection_name not in collection_names:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=self.vector_size,
                        distance=Distance.COSINE
                    )
                )
        except Exception as e:
            raise QdrantError(f"Failed to ensure collection: {str(e)}") from e

    async def search_chunks(
        self,
        query_vector: List[float],
        top_k: int = 3
    ) -> List[Dict[str, Any]]:
        """Search for similar chunks (async).

        Args:
            query_vector: 1024-dimensional query embedding
            top_k: Number of results to return

        Returns:
            List of dicts with keys: chunk_id, score, chapter, section, page, text

        Raises:
            QdrantError: If search fails
        """
        try:
            import asyncio
            loop = asyncio.get_event_loop()

            results = await loop.run_in_executor(
                None,
                lambda: self.client.search(
                    collection_name=self.collection_name,
                    query_vector=query_vector,
                    limit=top_k,
                    with_payload=True
                )
            )

            chunks = []
            for result in results:
                chunks.append({
                    "chunk_id": UUID(result.id),
                    "score": result.score,
                    "chapter": result.payload.get("chapter"),
                    "section": result.payload.get("section"),
                    "page": result.payload.get("page"),
                    "text": result.payload.get("text"),
                })

            return chunks
        except Exception as e:
            raise QdrantError(f"Failed to search chunks: {str(e)}") from e

    def search_chunks_sync(
        self,
        query_vector: List[float],
        top_k: int = 3
    ) -> List[Dict[str, Any]]:
        """Search for similar chunks (sync version for scripts).

        Args:
            query_vector: 1024-dimensional query embedding
            top_k: Number of results to return

        Returns:
            List of dicts with keys: chunk_id, score, chapter, section, page, text

        Raises:
            QdrantError: If search fails
        """
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                with_payload=True
            )

            chunks = []
            for result in results:
                chunks.append({
                    "chunk_id": UUID(result.id),
                    "score": result.score,
                    "chapter": result.payload.get("chapter"),
                    "section": result.payload.get("section"),
                    "page": result.payload.get("page"),
                    "text": result.payload.get("text"),
                })

            return chunks
        except Exception as e:
            raise QdrantError(f"Failed to search chunks: {str(e)}") from e

    def upsert_chunks(
        self,
        chunks: List[Dict[str, Any]]
    ) -> None:
        """Insert or update chunks in the vector store.

        Args:
            chunks: List of dicts with keys: chunk_id, embedding, chapter, section,
                   page, text, book_id, chunk_index, token_count

        Raises:
            QdrantError: If upsert fails
        """
        try:
            points = []
            for chunk in chunks:
                point = PointStruct(
                    id=str(chunk["chunk_id"]),
                    vector=chunk["embedding"],
                    payload={
                        "book_id": str(chunk.get("book_id", "")),
                        "chapter": chunk.get("chapter"),
                        "section": chunk.get("section"),
                        "page": chunk.get("page"),
                        "chunk_index": chunk.get("chunk_index"),
                        "text": chunk.get("text"),
                        "token_count": chunk.get("token_count"),
                    }
                )
                points.append(point)

            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
        except Exception as e:
            raise QdrantError(f"Failed to upsert chunks: {str(e)}") from e


# Singleton instance
vector_store_service = VectorStoreService()
