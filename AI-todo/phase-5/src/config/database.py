from sqlmodel import create_engine, Session
import os
from typing import Generator

# Database configuration - use SQLite for local development
# For production with Neon, set DATABASE_URL with asyncpg driver
DATABASE_URL = os.getenv("DATABASE_URL", "sqlite:///./phase5.db")

# For PostgreSQL, use psycopg2 or asyncpg
# If DATABASE_URL starts with postgresql://, ensure psycopg2 is installed
if DATABASE_URL.startswith("postgresql"):
    try:
        import psycopg2
    except ImportError:
        # Fallback to asyncpg if psycopg2 not available
        DATABASE_URL = DATABASE_URL.replace("postgresql://", "postgresql+asyncpg://")

# Create the database engine
engine = create_engine(DATABASE_URL, echo=True)


def get_session() -> Generator[Session, None, None]:
    with Session(engine) as session:
        yield session