"""Analytics API endpoints.

Provides query statistics and insights from the database.
"""
from fastapi import APIRouter, HTTPException, Query
from typing import Optional
from datetime import datetime
from src.services.database import database_service

router = APIRouter()


@router.get("/stats")
async def get_stats(
    start_date: Optional[str] = Query(None, description="Start date (ISO format)"),
    end_date: Optional[str] = Query(None, description="End date (ISO format)")
):
    """Get query statistics.

    Args:
        start_date: Optional start date (ISO 8601 format)
        end_date: Optional end date (ISO 8601 format)

    Returns:
        Dict with total_queries, avg_latency_ms, success_rate

    Raises:
        HTTPException: 400 for invalid dates, 500 for server errors
    """
    try:
        # Parse dates if provided
        start_dt = None
        end_dt = None

        if start_date:
            try:
                start_dt = datetime.fromisoformat(start_date)
            except ValueError:
                raise HTTPException(
                    status_code=400,
                    detail=f"Invalid start_date format: {start_date}"
                )

        if end_date:
            try:
                end_dt = datetime.fromisoformat(end_date)
            except ValueError:
                raise HTTPException(
                    status_code=400,
                    detail=f"Invalid end_date format: {end_date}"
                )

        # Get stats from database
        stats = await database_service.get_query_stats(
            start_date=start_dt,
            end_date=end_dt
        )

        return stats

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to get statistics: {str(e)}"
        )


@router.get("/errors")
async def get_top_errors(
    limit: int = Query(10, ge=1, le=100, description="Number of errors to return")
):
    """Get most common errors.

    Args:
        limit: Number of errors to return (1-100)

    Returns:
        List of dicts with error_code and count

    Raises:
        HTTPException: 500 for server errors
    """
    try:
        errors = await database_service.get_top_errors(limit=limit)
        return {"errors": errors, "total": len(errors)}

    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to get errors: {str(e)}"
        )
