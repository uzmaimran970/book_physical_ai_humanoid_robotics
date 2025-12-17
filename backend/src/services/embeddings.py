"""Cohere embeddings service.

Provides text embedding functionality using Cohere's embed-english-v3.0 model.
"""
import cohere
import asyncio
from typing import List
from concurrent.futures import ThreadPoolExecutor
from src.utils.config import config
from src.utils.error_handling import CohereError


class EmbeddingService:
    """Wrapper for Cohere embeddings API."""

    def __init__(self):
        """Initialize Cohere client."""
        self.client = cohere.Client(config.COHERE_API_KEY)
        self.model = config.COHERE_MODEL
        self.input_type_search = "search_query"
        self.input_type_document = "search_document"
        self.executor = ThreadPoolExecutor(max_workers=4)

    async def embed_query(self, query: str) -> List[float]:
        """Embed a search query (async).

        Args:
            query: User's question text

        Returns:
            1024-dimensional embedding vector

        Raises:
            CohereError: If embedding fails
        """
        try:
            loop = asyncio.get_event_loop()
            response = await loop.run_in_executor(
                self.executor,
                lambda: self.client.embed(
                    texts=[query],
                    model=self.model,
                    input_type=self.input_type_search
                )
            )
            return response.embeddings[0]
        except Exception as e:
            raise CohereError(f"Failed to embed query: {str(e)}") from e

    def embed_query_sync(self, query: str) -> List[float]:
        """Embed a search query (sync version for scripts).

        Args:
            query: User's question text

        Returns:
            1024-dimensional embedding vector

        Raises:
            CohereError: If embedding fails
        """
        try:
            response = self.client.embed(
                texts=[query],
                model=self.model,
                input_type=self.input_type_search
            )
            return response.embeddings[0]
        except Exception as e:
            raise CohereError(f"Failed to embed query: {str(e)}") from e

    async def embed_chunks(self, texts: List[str]) -> List[List[float]]:
        """Embed multiple text chunks (async).

        Args:
            texts: List of text chunks to embed

        Returns:
            List of 1024-dimensional embedding vectors

        Raises:
            CohereError: If embedding fails
        """
        try:
            loop = asyncio.get_event_loop()
            response = await loop.run_in_executor(
                self.executor,
                lambda: self.client.embed(
                    texts=texts,
                    model=self.model,
                    input_type=self.input_type_document
                )
            )
            return response.embeddings
        except Exception as e:
            raise CohereError(f"Failed to embed chunks: {str(e)}") from e

    def embed_chunks_sync(self, texts: List[str]) -> List[List[float]]:
        """Embed multiple text chunks (sync version for scripts).

        Args:
            texts: List of text chunks to embed

        Returns:
            List of 1024-dimensional embedding vectors

        Raises:
            CohereError: If embedding fails
        """
        try:
            response = self.client.embed(
                texts=texts,
                model=self.model,
                input_type=self.input_type_document
            )
            return response.embeddings
        except Exception as e:
            raise CohereError(f"Failed to embed chunks: {str(e)}") from e


# Singleton instance
embedding_service = EmbeddingService()
