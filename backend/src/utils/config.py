"""Configuration management for RAG Chatbot.

Loads environment variables and provides configuration access.
"""
import os
from typing import Optional
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()


class Config:
    """Application configuration."""

    # Qdrant
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_COLLECTION: str = "textbook_chunks"

    # Cohere
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")
    COHERE_MODEL: str = "embed-english-v3.0"

    # Anthropic Claude
    ANTHROPIC_API_KEY: str = os.getenv("ANTHROPIC_API_KEY", "")
    CLAUDE_MODEL: str = "claude-sonnet-4-20250514"

    # Neon DB
    NEON_DB_URL: str = os.getenv("NEON_DB_URL", "")

    # Application settings
    CHUNK_SIZE: int = int(os.getenv("CHUNK_SIZE", "750"))
    CHUNK_OVERLAP: int = int(os.getenv("CHUNK_OVERLAP", "75"))
    TOP_K_DEFAULT: int = int(os.getenv("TOP_K_DEFAULT", "3"))
    MAX_QUERY_LENGTH: int = int(os.getenv("MAX_QUERY_LENGTH", "2000"))
    LOG_LEVEL: str = os.getenv("LOG_LEVEL", "INFO")

    # Frontend
    FRONTEND_URL: str = os.getenv("FRONTEND_URL", "http://localhost:3000")

    @classmethod
    def validate(cls) -> None:
        """Validate required configuration."""
        required = [
            ("QDRANT_URL", cls.QDRANT_URL),
            ("QDRANT_API_KEY", cls.QDRANT_API_KEY),
            ("COHERE_API_KEY", cls.COHERE_API_KEY),
            ("ANTHROPIC_API_KEY", cls.ANTHROPIC_API_KEY),
            ("NEON_DB_URL", cls.NEON_DB_URL),
        ]

        missing = [name for name, value in required if not value]
        if missing:
            raise ValueError(f"Missing required environment variables: {', '.join(missing)}")


config = Config()
