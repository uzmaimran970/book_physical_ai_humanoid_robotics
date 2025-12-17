"""RAG pipeline orchestration.

Integrates embeddings, vector search, LLM generation, and query logging.
"""
from typing import Dict, Any, List, Optional
from uuid import uuid4
from datetime import datetime
from src.services.embeddings import embedding_service
from src.services.vector_store import vector_store_service
from src.services.llm_simple import simple_llm_service  # Using simple formatter for user-friendly responses
from src.services.database import database_service
from src.models.entities import QueryRequest, QueryResponse, Citation
from src.utils.query_classifier import QueryClassifier


class RAGPipeline:
    """Orchestrates the complete RAG pipeline with logging."""

    def __init__(self):
        """Initialize pipeline with service dependencies."""
        self.embedding_service = embedding_service
        self.vector_store = vector_store_service
        self.llm_service = simple_llm_service  # Using simple, user-friendly formatter
        self.database_service = database_service

    async def query(
        self,
        request: QueryRequest,
        enable_logging: bool = True
    ) -> QueryResponse:
        """Execute the complete RAG pipeline with optional logging.

        Args:
            request: QueryRequest with query text and top_k
            enable_logging: Whether to log to database (default: True)

        Returns:
            QueryResponse with answer, citations, and metadata
        """
        query_id = uuid4()
        response_id = uuid4()
        start_time = datetime.now()

        # Check if it's a greeting
        if QueryClassifier.is_greeting(request.query):
            greeting_response = QueryClassifier.get_greeting_response(request.query)
            end_time = datetime.now()
            latency = int((end_time - start_time).total_seconds() * 1000)

            return QueryResponse(
                query_id=query_id,
                answer=greeting_response["answer"],
                citations=[],
                latency_ms=latency,
                status=greeting_response["status"]
            )

        # Check if query is too short
        if QueryClassifier.is_too_short(request.query):
            end_time = datetime.now()
            latency = int((end_time - start_time).total_seconds() * 1000)

            return QueryResponse(
                query_id=query_id,
                answer="I need more information to help you. Could you please ask a more specific question about robotics or the textbook content?",
                citations=[],
                latency_ms=latency,
                status="clarification_needed"
            )

        try:
            # Step 1: Log the incoming query (non-blocking)
            if enable_logging:
                try:
                    await self.database_service.log_query(
                        query_id=query_id,
                        query_text=request.query,
                        top_k=request.top_k,
                        timestamp=start_time
                    )
                except Exception as log_err:
                    # Don't fail the query if logging fails
                    print(f"Warning: Failed to log query: {log_err}")

            # Step 2: Embed the query
            query_vector = await self.embedding_service.embed_query(request.query)

            # Step 3: Search for similar chunks
            retrieved_chunks = await self.vector_store.search_chunks(
                query_vector=query_vector,
                top_k=request.top_k
            )

            # Step 4: Log retrieved chunks (non-blocking)
            if enable_logging and retrieved_chunks:
                try:
                    chunks_for_logging = [
                        {
                            "chunk_id": chunk["chunk_id"],
                            "score": chunk["score"],
                            "rank": idx + 1
                        }
                        for idx, chunk in enumerate(retrieved_chunks)
                    ]
                    await self.database_service.log_retrieved_chunks(
                        query_id=query_id,
                        chunks=chunks_for_logging
                    )
                except Exception as log_err:
                    print(f"Warning: Failed to log retrieved chunks: {log_err}")

            # Step 5: Generate response with citations
            llm_result = await self.llm_service.generate_response(
                query=request.query,
                retrieved_chunks=retrieved_chunks
            )

            # Step 6: Extract citations
            citations = self._build_citations(retrieved_chunks)

            # Calculate latency
            end_time = datetime.now()
            latency_ms = int((end_time - start_time).total_seconds() * 1000)

            # Step 7: Log the response (non-blocking)
            if enable_logging:
                try:
                    citations_json = [
                        {
                            "chapter": c.chapter,
                            "section": c.section,
                            "page": c.page,
                            "chunk_id": str(c.chunk_id),
                            "relevance_score": c.relevance_score
                        }
                        for c in citations
                    ]

                    await self.database_service.log_response(
                        response_id=response_id,
                        query_id=query_id,
                        response_text=llm_result["answer"],
                        citations=citations_json,
                        latency_ms=latency_ms,
                        status=llm_result["status"]
                    )
                except Exception as log_err:
                    print(f"Warning: Failed to log response: {log_err}")

            return QueryResponse(
                query_id=query_id,
                answer=llm_result["answer"],
                citations=citations,
                latency_ms=latency_ms,
                status=llm_result["status"]
            )

        except Exception as e:
            # Log the error
            if enable_logging:
                try:
                    await self.database_service.log_error(
                        error_id=uuid4(),
                        query_id=query_id,
                        error_code=type(e).__name__,
                        error_message=str(e),
                        timestamp=datetime.now()
                    )
                except Exception:
                    pass  # Don't fail on error logging

            # Re-raise the original error
            raise

    def _build_citations(self, chunks: List[Dict[str, Any]]) -> List[Citation]:
        """Build citation objects from retrieved chunks.

        Args:
            chunks: List of retrieved chunk dicts

        Returns:
            List of Citation objects
        """
        citations = []
        for chunk in chunks:
            citation = Citation(
                chapter=chunk.get("chapter", 1),
                section=chunk.get("section"),
                page=chunk.get("page"),
                chunk_id=chunk["chunk_id"],
                relevance_score=chunk["score"]
            )
            citations.append(citation)

        return citations


# Singleton instance
rag_pipeline = RAGPipeline()
