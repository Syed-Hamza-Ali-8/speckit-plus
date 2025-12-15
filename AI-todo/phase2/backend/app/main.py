"""FastAPI application entry point with lifespan management."""

from contextlib import asynccontextmanager
from collections.abc import AsyncGenerator

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from slowapi import _rate_limit_exceeded_handler
from slowapi.errors import RateLimitExceeded

from app.api.routes import auth, notifications, tasks
from app.core.database import close_db, get_async_engine
from app.middleware.rate_limit import limiter


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    """Application lifespan context manager.

    Handles startup and shutdown events:
    - Startup: Initialize database engine
    - Shutdown: Close database connections
    """
    # Startup: ensure engine is created
    get_async_engine()
    yield
    # Shutdown: close database connections
    await close_db()


# T030: Create FastAPI app with lifespan
app = FastAPI(
    title="Todo API",
    description="Multi-user Todo application with JWT authentication",
    version="1.0.0",
    lifespan=lifespan,
    docs_url="/docs",
    redoc_url="/redoc",
    openapi_url="/openapi.json",
)

# T033: Add slowapi rate limit middleware and error handler
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# T034: Configure CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify allowed origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# T031: Mount auth router at /auth prefix
app.include_router(auth.router)

# T032: Mount tasks router at /tasks prefix
app.include_router(tasks.router)

# Mount notifications router at /notifications prefix
app.include_router(notifications.router)


@app.get("/", tags=["root"])
async def root() -> dict[str, str]:
    """Root endpoint returning API information."""
    return {
        "name": "Todo API",
        "version": "1.0.0",
        "docs": "/docs",
    }


@app.get("/health", tags=["health"])
async def health_check() -> dict[str, str]:
    """Health check endpoint for monitoring."""
    return {"status": "healthy"}
