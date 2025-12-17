"""Claude LLM service.

Provides text generation functionality using Anthropic's Claude API.
"""
from typing import List, Dict, Any
from anthropic import Anthropic
from src.utils.config import config
from src.utils.error_handling import ClaudeError


class LLMService:
    """Wrapper for Anthropic Claude API."""

    def __init__(self):
        """Initialize Anthropic client."""
        self.client = Anthropic(api_key=config.ANTHROPIC_API_KEY)
        self.model = config.CLAUDE_MODEL
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
            ClaudeError: If generation fails
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

        # Create system prompt
        system_prompt = """You are an expert assistant for a textbook on Physical AI & Humanoid Robotics.

Answer questions ONLY using the provided context from the book. You must:
1. Base your answer strictly on the retrieved passages
2. Include inline citations in the format [Chapter X, Section Y, p. Z]
3. If the context doesn't contain enough information, respond: "I don't have enough information to answer this question based on the available content."
4. Be accurate and concise
5. Do not make up information or draw from outside knowledge"""

        # Create user prompt
        user_prompt = f"""Context from the textbook:

{context}

Question: {query}

Please answer the question using ONLY the context provided above. Include citations in your answer."""

        try:
            import asyncio
            loop = asyncio.get_event_loop()

            message = await loop.run_in_executor(
                None,
                lambda: self.client.messages.create(
                    model=self.model,
                    max_tokens=self.max_tokens,
                    system=system_prompt,
                    messages=[
                        {"role": "user", "content": user_prompt}
                    ]
                )
            )

            answer = message.content[0].text

            # Determine status based on answer content
            if "don't have enough information" in answer.lower():
                status = "cannot_answer"
            elif len(retrieved_chunks) < 2:
                status = "partial"
            else:
                status = "success"

            return {
                "answer": answer,
                "status": status
            }

        except Exception as e:
            raise ClaudeError(f"Failed to generate response: {str(e)}") from e

    def _format_citation(self, chunk: Dict[str, Any]) -> str:
        """Format a citation string from chunk metadata.

        Args:
            chunk: Dict with chapter, section, page

        Returns:
            Formatted citation string
        """
        parts = []

        if chunk.get("chapter"):
            parts.append(f"Chapter {chunk['chapter']}")

        if chunk.get("section"):
            parts.append(chunk["section"])

        if chunk.get("page"):
            parts.append(f"p. {chunk['page']}")

        return ", ".join(parts) if parts else "Unknown source"


# Singleton instance
llm_service = LLMService()
