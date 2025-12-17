"""Custom exceptions for RAG Chatbot."""


class RAGError(Exception):
    """Base exception for RAG chatbot errors."""
    pass


class CohereError(RAGError):
    """Exception for Cohere API errors."""
    pass


class QdrantError(RAGError):
    """Exception for Qdrant operations errors."""
    pass


class ClaudeError(RAGError):
    """Exception for Claude API errors."""
    pass


class ValidationError(RAGError):
    """Exception for input validation errors."""
    pass


class DatabaseError(RAGError):
    """Exception for database operations errors."""
    pass
