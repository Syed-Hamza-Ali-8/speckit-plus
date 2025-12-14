"""Async database engine and session factory for Neon Postgres."""

from collections.abc import AsyncGenerator
from typing import Annotated

from sqlalchemy.ext.asyncio import AsyncEngine, async_sessionmaker, create_async_engine
from sqlmodel import SQLModel
from sqlmodel.ext.asyncio.session import AsyncSession

from app.core.config import settings

# Module-level engine instance (lazy initialization)
_engine: AsyncEngine | None = None


def get_async_engine() -> AsyncEngine:
    """Create and return the async database engine.

    Returns:
        AsyncEngine configured for Neon Postgres with SSL.

    Note:
        Uses singleton pattern - returns same engine on subsequent calls.
        Connection string must include sslmode=require for Neon.
    """
    global _engine
    if _engine is None:
        _engine = create_async_engine(
            settings.database_url,
            echo=False,
        )
    return _engine


def get_async_session_factory() -> async_sessionmaker[AsyncSession]:
    """Create async session factory.

    Returns:
        Configured async_sessionmaker for creating sessions.
    """
    engine = get_async_engine()
    return async_sessionmaker(
        engine,
        class_=AsyncSession,
        expire_on_commit=False,
    )


async def get_async_session() -> AsyncGenerator[AsyncSession, None]:
    """Yield an async database session for dependency injection.

    Yields:
        AsyncSession with transaction context.

    Note:
        - Session auto-commits on successful yield
        - Session auto-rollbacks on exception
        - expire_on_commit=False prevents implicit I/O
    """
    session_factory = get_async_session_factory()
    async with session_factory() as session:
        yield session


async def init_db() -> None:
    """Initialize database tables.

    Note:
        For development/testing only. Production uses Alembic migrations.
    """
    engine = get_async_engine()
    async with engine.begin() as conn:
        await conn.run_sync(SQLModel.metadata.create_all)


async def close_db() -> None:
    """Close database engine connections."""
    global _engine
    if _engine is not None:
        await _engine.dispose()
        _engine = None
