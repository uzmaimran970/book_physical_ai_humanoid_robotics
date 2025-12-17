"""API route handlers.

Defines HTTP endpoints for the RAG chatbot.
"""
from fastapi import APIRouter, HTTPException
from datetime import datetime
from src.models.entities import QueryRequest, QueryResponse, HealthResponse, ErrorResponse
from src.services.rag_pipeline import rag_pipeline
from src.services.vector_store import vector_store_service
from src.utils.error_handling import RAGError, CohereError, QdrantError, ClaudeError

router = APIRouter()


@router.post("/ask", response_model=QueryResponse)
@router.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest) -> QueryResponse:
    """Query the textbook using RAG pipeline.

    Args:
        request: QueryRequest with query text and optional top_k

    Returns:
        QueryResponse with answer, citations, and metadata

    Raises:
        HTTPException: 400 for invalid input, 500 for server errors
    """
    try:
        # Validate query length
        if len(request.query.strip()) == 0:
            raise HTTPException(
                status_code=400,
                detail="Query cannot be empty"
            )

        # Execute RAG pipeline (now async)
        response = await rag_pipeline.query(request, enable_logging=True)
        return response

    except CohereError as e:
        raise HTTPException(
            status_code=500,
            detail=f"Embedding service error: {str(e)}"
        )
    except QdrantError as e:
        raise HTTPException(
            status_code=500,
            detail=f"Vector store error: {str(e)}"
        )
    except ClaudeError as e:
        raise HTTPException(
            status_code=500,
            detail=f"LLM service error: {str(e)}"
        )
    except RAGError as e:
        raise HTTPException(
            status_code=500,
            detail=f"RAG pipeline error: {str(e)}"
        )
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Unexpected error: {str(e)}"
        )


@router.get("/health", response_model=HealthResponse)
async def health_check() -> HealthResponse:
    """Health check endpoint.

    Returns:
        HealthResponse with service status

    Raises:
        HTTPException: 503 if services are unavailable
    """
    services = {}
    overall_status = "healthy"

    # Check Qdrant
    try:
        vector_store_service.client.get_collections()
        services["qdrant"] = "healthy"
    except Exception as e:
        services["qdrant"] = f"down: {str(e)}"
        overall_status = "degraded"

    # Check embeddings (basic connectivity)
    try:
        # Just check if we have API key configured
        from src.utils.config import config
        if config.COHERE_API_KEY:
            services["cohere"] = "healthy"
        else:
            services["cohere"] = "down: API key not configured"
            overall_status = "degraded"
    except Exception as e:
        services["cohere"] = f"down: {str(e)}"
        overall_status = "degraded"

    # Check Claude
    try:
        from src.utils.config import config
        if config.ANTHROPIC_API_KEY:
            services["claude"] = "healthy"
        else:
            services["claude"] = "down: API key not configured"
            overall_status = "degraded"
    except Exception as e:
        services["claude"] = f"down: {str(e)}"
        overall_status = "degraded"

    return HealthResponse(
        status=overall_status,
        timestamp=datetime.now().isoformat(),
        services=services
    )
