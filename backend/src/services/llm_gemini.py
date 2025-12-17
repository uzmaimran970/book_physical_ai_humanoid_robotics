"""Google Gemini LLM service (FREE alternative to Claude).

Provides text generation using Google's Gemini API (free tier: 1500 requests/day).
"""
from typing import List, Dict, Any
import google.generativeai as genai
from src.utils.config import config
import os


class GeminiLLMService:
    """Wrapper for Google Gemini API."""

    def __init__(self):
        """Initialize Gemini client."""
        api_key = os.getenv("GEMINI_API_KEY", "")
        if api_key:
            genai.configure(api_key=api_key)
            self.model = genai.GenerativeModel('gemini-pro')  # Free tier model
            self.enabled = True
        else:
            self.enabled = False
            print("⚠️ GEMINI_API_KEY not set - using simple formatter")

    async def generate_response(
        self,
        query: str,
        retrieved_chunks: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """Generate a response with citations.

        Args:
            query: User's question
            retrieved_chunks: List of dicts with keys: chunk_id, score, chapter,
                            section, page, text

        Returns:
            Dict with keys:
                - answer: Generated response text with inline citations
                - status: "success", "partial", or "cannot_answer"
        """
        if not retrieved_chunks:
            return {
                "answer": "I don't have enough information to answer this question.",
                "status": "cannot_answer"
            }

        # If Gemini not configured, return simple formatted response
        if not self.enabled:
            return self._simple_format(query, retrieved_chunks)

        # Build context from retrieved chunks
        context_parts = []
        for idx, chunk in enumerate(retrieved_chunks, 1):
            citation = self._format_citation(chunk)
            context_parts.append(
                f"[Source {idx}] {citation}\n{chunk['text']}\n"
            )

        context = "\n\n".join(context_parts)

        # Create prompt
        prompt = f"""You are an expert assistant for a textbook on Physical AI & Humanoid Robotics.

Answer questions ONLY using the provided context from the book. You must:
1. Base your answer strictly on the retrieved passages
2. Include inline citations in the format [Chapter X, Section Y, p. Z]
3. If the context doesn't contain enough information, respond: "I don't have enough information to answer this question based on the available content."
4. Be accurate and concise
5. Do not make up information or draw from outside knowledge

Context from the textbook:

{context}

Question: {query}

Please answer the question using ONLY the context provided above. Include citations in your answer."""

        try:
            import asyncio
            loop = asyncio.get_event_loop()

            response = await loop.run_in_executor(
                None,
                lambda: self.model.generate_content(
                    prompt,
                    generation_config={
                        'temperature': 0.3,
                        'max_output_tokens': 1024,
                    }
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
            print(f"⚠️ Gemini error: {e}")
            # Fallback to simple format on error
            return self._simple_format(query, retrieved_chunks)

    def _simple_format(self, query: str, retrieved_chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Fallback: Simple formatting without LLM."""
        answer_parts = ["Based on the textbook, here's what I found:\n"]

        for idx, chunk in enumerate(retrieved_chunks, 1):
            citation = self._format_citation(chunk)
            relevance_pct = int(chunk.get('score', 0) * 100)

            answer_parts.append(
                f"\n**{citation}** (relevance: {relevance_pct}%)\n{chunk['text']}"
            )

        return {
            "answer": "\n".join(answer_parts),
            "status": "success"
        }

    def _format_citation(self, chunk: Dict[str, Any]) -> str:
        """Format a citation from chunk metadata."""
        chapter = chunk.get("chapter", "?")
        section = chunk.get("section", "?")
        page = chunk.get("page", "?")
        return f"Chapter {chapter}, Section {section}, p. {page}"


# Singleton instance
gemini_llm_service = GeminiLLMService()
