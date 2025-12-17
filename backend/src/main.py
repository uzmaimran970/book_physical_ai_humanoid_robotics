"""FastAPI application entry point.

Main application setup and configuration with database lifecycle management.
"""
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.api.routes import router
from src.api.analytics import router as analytics_router
from src.utils.config import config
from src.services.database import database_service

# Validate configuration on startup
config.validate()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan manager.

    Handles startup and shutdown events for database connection pool.
    """
    # Startup: Initialize database connection pool
    try:
        await database_service.initialize()
        print("✓ Database connection pool initialized")
    except Exception as e:
        print(f"⚠ Database initialization failed: {e}")
        print("  Continuing without database logging...")

    yield

    # Shutdown: Close database connection pool
    try:
        await database_service.close()
        print("✓ Database connection pool closed")
    except Exception as e:
        print(f"⚠ Database cleanup failed: {e}")


# Create FastAPI app with lifespan
app = FastAPI(
    title="RAG Chatbot API",
    description="Retrieval-Augmented Generation chatbot for Physical AI textbook",
    version="1.0.0",
    lifespan=lifespan
)

# Configure CORS for Vercel frontend
allowed_origins = [
    "http://localhost:3000",
    "http://localhost:8000",
    config.FRONTEND_URL,
    "https://book-physical-ai-humanoid-robotics.vercel.app",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routes
app.include_router(router, prefix="/api/v1", tags=["rag"])
app.include_router(analytics_router, prefix="/api/v1/analytics", tags=["analytics"])


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "RAG Chatbot API",
        "version": "1.0.0",
        "docs": "/docs"
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
