"""Cohere LLM service (fallback/alternative to Claude).

Provides text generation functionality using Cohere's Chat API.
"""
from typing import List, Dict, Any
import cohere
from src.utils.config import config
from src.utils.error_handling import CohereError


class CohereLLMService:
    """Wrapper for Cohere Chat API."""

    def __init__(self):
        """Initialize Cohere client."""
        self.client = cohere.Client(config.COHERE_API_KEY)
        self.model = "command"  # Cohere base command model
        self.max_tokens = 1024

    async def generate_response(
        self,
        query: str,
        retrieved_chunks: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """Generate a response with citations (async).

        Args:
            query: User's question
            retrieved_chunks: List of dicts with keys: chunk_id, score, chapter,
                            section, page, text

        Returns:
            Dict with keys:
                - answer: Generated response text with inline citations
                - status: "success", "partial", or "cannot_answer"

        Raises:
            CohereError: If generation fails
        """
        if not retrieved_chunks:
            return {
                "answer": "I don't have enough information to answer this question.",
                "status": "cannot_answer"
            }

        # Build context from retrieved chunks
        context_parts = []
        for idx, chunk in enumerate(retrieved_chunks, 1):
            citation = self._format_citation(chunk)
            context_parts.append(
                f"[Source {idx}] {citation}\n{chunk['text']}\n"
            )

        context = "\n\n".join(context_parts)

        # Create system preamble and user message for Chat API
        preamble = """You are an expert assistant for a textbook on Physical AI & Humanoid Robotics.

Answer questions ONLY using the provided context from the book. You must:
1. Base your answer strictly on the retrieved passages
2. Include inline citations in the format [Chapter X, Section Y, p. Z]
3. If the context doesn't contain enough information, respond: "I don't have enough information to answer this question based on the available content."
4. Be accurate and concise
5. Do not make up information or draw from outside knowledge"""

        user_message = f"""Context from the textbook:

{context}

Question: {query}

Please answer the question using ONLY the context provided above. Include citations in your answer."""

        try:
            import asyncio
            loop = asyncio.get_event_loop()

            response = await loop.run_in_executor(
                None,
                lambda: self.client.chat(
                    message=user_message,
                    preamble_override=preamble,
                    model=self.model,
                    max_tokens=self.max_tokens,
                    temperature=0.3
                )
            )

            answer_text = response.text

            # Determine status
            if "don't have enough information" in answer_text.lower():
                status = "cannot_answer"
            else:
                status = "success"

            return {
                "answer": answer_text,
                "status": status
            }

        except Exception as e:
            raise CohereError(f"Failed to generate response: {str(e)}") from e

    def _format_citation(self, chunk: Dict[str, Any]) -> str:
        """Format a citation from chunk metadata.

        Args:
            chunk: Chunk dict with metadata

        Returns:
            Formatted citation string
        """
        chapter = chunk.get("chapter", "?")
        section = chunk.get("section", "?")
        page = chunk.get("page", "?")
        return f"Chapter {chapter}, Section {section}, p. {page}"


# Singleton instance
cohere_llm_service = CohereLLMService()
