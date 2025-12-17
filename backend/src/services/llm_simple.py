"""Simple response service (no LLM needed).

Provides basic responses by formatting retrieved chunks without LLM generation.
"""
from typing import List, Dict, Any


class SimpleLLMService:
    """Simple response formatter that doesn't require an LLM API."""

    # Minimum relevance threshold to consider answer valid
    RELEVANCE_THRESHOLD = 0.40  # 40%

    def __init__(self):
        """Initialize service."""
        pass

    async def generate_response(
        self,
        query: str,
        retrieved_chunks: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """Generate a response by formatting retrieved chunks.

        Args:
            query: User's question
            retrieved_chunks: List of dicts with keys: chunk_id, score, chapter,
                            section, page, text

        Returns:
            Dict with keys:
                - answer: Formatted response with chunks
                - status: "success" or "cannot_answer"
        """
        if not retrieved_chunks:
            return {
                "answer": "I don't have information about this topic in my textbook.",
                "status": "cannot_answer"
            }

        # Check if the top result is relevant enough
        top_score = retrieved_chunks[0].get('score', 0)
        if top_score < self.RELEVANCE_THRESHOLD:
            return {
                "answer": "Sorry, this topic is not covered in my Physical AI & Humanoid Robotics textbook. I can only answer questions about robotics, AI systems, sensors, control, and related topics from the book.",
                "status": "cannot_answer"
            }

        # Format answer in a user-friendly way (no citations, no technical details)
        answer_parts = []

        for idx, chunk in enumerate(retrieved_chunks, 1):
            # Just add the text content, no citations or metadata
            text = chunk.get('text', '')

            # Break long text into paragraphs for readability
            if len(text) > 200:
                # Add first chunk with intro
                if idx == 1:
                    answer_parts.append(text)
                else:
                    # Add additional info
                    answer_parts.append(f"\n{text}")
            else:
                answer_parts.append(text)

        # Combine all parts
        answer = "\n\n".join(answer_parts)

        # Make it more friendly by adding a helpful intro
        friendly_answer = f"{answer}\n\nNeed more details? Feel free to ask!"

        return {
            "answer": friendly_answer,
            "status": "success"
        }

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
simple_llm_service = SimpleLLMService()
