"""Pydantic models for API requests and responses."""
from typing import List, Optional, Literal
from pydantic import BaseModel, Field
from uuid import UUID


class QueryRequest(BaseModel):
    """Request model for /query endpoint."""
    query: str = Field(..., min_length=1, max_length=2000, description="User's question")
    top_k: int = Field(default=3, ge=1, le=10, description="Number of chunks to retrieve")


class Citation(BaseModel):
    """Citation information for a source."""
    chapter: int = Field(..., ge=1, description="Chapter number")
    section: Optional[str] = Field(None, description="Section title")
    page: Optional[int] = Field(None, ge=1, description="Page number")
    chunk_id: UUID = Field(..., description="Source chunk ID")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Cosine similarity score")


class QueryResponse(BaseModel):
    """Response model for /query endpoint."""
    query_id: UUID = Field(..., description="Unique query identifier")
    answer: str = Field(..., description="Generated response with citations")
    citations: List[Citation] = Field(default_factory=list, description="List of sources")
    latency_ms: int = Field(..., ge=0, description="Processing time in milliseconds")
    status: Literal["success", "partial", "cannot_answer", "greeting", "clarification_needed"] = Field(..., description="Query outcome")


class ErrorResponse(BaseModel):
    """Error response model."""
    error_code: str = Field(..., description="Machine-readable error code")
    message: str = Field(..., description="Human-readable error message")
    timestamp: str = Field(..., description="Error timestamp (ISO 8601)")


class HealthResponse(BaseModel):
    """Health check response model."""
    status: Literal["healthy", "degraded", "down"] = Field(..., description="Overall status")
    timestamp: str = Field(..., description="Health check timestamp")
    services: dict = Field(..., description="Individual service statuses")
