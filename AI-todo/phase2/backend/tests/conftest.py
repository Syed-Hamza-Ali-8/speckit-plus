"""Pytest configuration and fixtures for unit and integration tests."""

import os
import ssl
from collections.abc import AsyncGenerator
from pathlib import Path
from uuid import uuid4

import pytest
from dotenv import load_dotenv
from httpx import ASGITransport, AsyncClient

# Load .env file from backend directory
_backend_dir = Path(__file__).parent.parent
load_dotenv(_backend_dir / ".env")

from sqlalchemy.ext.asyncio import AsyncEngine, async_sessionmaker, create_async_engine
from sqlalchemy.pool import NullPool
from sqlmodel import SQLModel
from sqlmodel.ext.asyncio.session import AsyncSession

from app.models.task import Task, TaskStatus
from app.models.user import User

# Enable pytest-asyncio plugin
pytest_plugins = ["pytest_asyncio"]

# ---------------------------------------------------------------------------
# SQLite Fixtures (Unit Tests)
# ---------------------------------------------------------------------------

@pytest.fixture(scope="function")
async def sqlite_engine() -> AsyncGenerator[AsyncEngine, None]:
    """Create SQLite in-memory async engine for unit tests."""
    engine = create_async_engine(
        "sqlite+aiosqlite:///:memory:",
        echo=False,
    )

    # Create all tables
    async with engine.begin() as conn:
        await conn.run_sync(SQLModel.metadata.create_all)

    yield engine
    await engine.dispose()


@pytest.fixture(scope="function")
async def sqlite_session(
    sqlite_engine: AsyncEngine,
) -> AsyncGenerator[AsyncSession, None]:
    """Create isolated SQLite session for unit tests.

    Uses fresh in-memory database per test (via sqlite_engine fixture)
    so no nested transactions needed for isolation.
    """
    session_factory = async_sessionmaker(
        sqlite_engine,
        class_=AsyncSession,
        expire_on_commit=False,
    )

    async with session_factory() as session:
        yield session

# ---------------------------------------------------------------------------
# Neon/Postgres Fixtures (Integration Tests)
# ---------------------------------------------------------------------------

@pytest.fixture(scope="function")
async def neon_engine() -> AsyncGenerator[AsyncEngine, None]:
    """Create Neon/Postgres async engine for integration tests.

    Uses NullPool to avoid event loop binding issues with asyncpg.
    Each test gets fresh connections that work with the current event loop.
    """
    test_db_url = os.environ.get("TEST_DATABASE_URL")
    if not test_db_url:
        pytest.skip("TEST_DATABASE_URL not set - skipping integration tests")

    # Remove ssl query param from URL if present
    clean_url = test_db_url.replace("?ssl=require", "").replace("&ssl=require", "")

    # SSL context for asyncpg
    ssl_context = ssl.create_default_context()
    ssl_context.check_hostname = False
    ssl_context.verify_mode = ssl.CERT_NONE

    engine = create_async_engine(
        clean_url,
        echo=False,
        connect_args={"ssl": ssl_context},
        poolclass=NullPool,  # Avoid event loop binding issues
    )

    # Ensure tables exist
    async with engine.begin() as conn:
        await conn.run_sync(SQLModel.metadata.create_all)

    yield engine
    await engine.dispose()


@pytest.fixture(scope="function")
async def neon_session(
    neon_engine: AsyncEngine,
) -> AsyncGenerator[AsyncSession, None]:
    """Create isolated Neon/Postgres session for integration tests.

    Uses a connection with a transaction that gets rolled back after each test,
    providing test isolation while sharing the session-scoped engine.
    """
    async with neon_engine.connect() as connection:
        # Start a transaction that we'll roll back at the end
        trans = await connection.begin()

        # Create session bound to this connection
        session = AsyncSession(bind=connection, expire_on_commit=False)
        try:
            yield session
        finally:
            await session.close()
            # Rollback the transaction - all test data is discarded
            await trans.rollback()

# ---------------------------------------------------------------------------
# Sample Data Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def sample_task() -> Task:
    """Create a valid Task instance for testing."""
    return Task(
        user_id=uuid4(),
        title="Test Task",
        description="Test Description",
        status=TaskStatus.PENDING,
    )


@pytest.fixture
def sample_user_id() -> str:
    """Generate a random UUID for user_id in tests."""
    return str(uuid4())


@pytest.fixture
def sample_user() -> User:
    """Create a valid User instance for testing."""
    return User(
        email="test@example.com",
        hashed_password="$argon2id$v=19$m=65536,t=2,p=1$fakesalt$fakehash",
        is_active=True,
    )


@pytest.fixture
async def test_user(neon_session: AsyncSession) -> User:
    """Create a test user in the database for foreign key relationships.

    Use this fixture when tests need to create tasks with valid user_id.
    """
    user = User(
        email=f"testuser_{uuid4().hex[:8]}@example.com",
        hashed_password="$argon2id$v=19$m=65536,t=2,p=1$fakesalt$fakehash",
        is_active=True,
    )
    neon_session.add(user)
    await neon_session.flush()
    return user


# ---------------------------------------------------------------------------
# HTTP Client Fixtures (Integration Tests)
# ---------------------------------------------------------------------------

@pytest.fixture
async def test_client(
    neon_session: AsyncSession,
) -> AsyncGenerator[AsyncClient, None]:
    """Create async HTTP client for integration tests.

    Uses the FastAPI app directly via ASGI transport.
    Injects the test session so all changes get rolled back.
    Disables rate limiting during tests.
    """
    from app.main import app
    from app.core.database import get_async_session
    from app.middleware.rate_limit import limiter

    # Override the session dependency to use our test session
    async def override_get_session():
        yield neon_session

    app.dependency_overrides[get_async_session] = override_get_session

    # Disable rate limiting during tests
    limiter.enabled = False

    transport = ASGITransport(app=app)
    async with AsyncClient(transport=transport, base_url="http://test") as client:
        yield client

    # Clean up
    app.dependency_overrides.clear()
    limiter.enabled = True


@pytest.fixture
async def test_client_with_rate_limit(
    neon_session: AsyncSession,
) -> AsyncGenerator[AsyncClient, None]:
    """Create async HTTP client with rate limiting enabled.

    Use this fixture specifically for tests that verify rate limiting behavior.
    """
    from app.main import app
    from app.core.database import get_async_session
    from app.middleware.rate_limit import limiter

    # Override the session dependency to use our test session
    async def override_get_session():
        yield neon_session

    app.dependency_overrides[get_async_session] = override_get_session

    # Keep rate limiting enabled for rate limit tests
    limiter.enabled = True
    # Reset the limiter storage to ensure clean state
    limiter.reset()

    transport = ASGITransport(app=app)
    async with AsyncClient(transport=transport, base_url="http://test") as client:
        yield client

    # Clean up
    app.dependency_overrides.clear()


@pytest.fixture
def unique_email() -> str:
    """Generate a unique email for each test."""
    return f"test_{uuid4().hex[:8]}@example.com"


@pytest.fixture
async def registered_user(test_client: AsyncClient, unique_email: str) -> dict:
    """Register a user and return their credentials."""
    password = "testpassword123"
    response = await test_client.post(
        "/auth/register",
        json={"email": unique_email, "password": password},
    )
    assert response.status_code == 201
    user_data = response.json()
    return {
        "id": user_data["id"],
        "email": unique_email,
        "password": password,
    }


@pytest.fixture
async def auth_token(test_client: AsyncClient, registered_user: dict) -> str:
    """Get an auth token for a registered user."""
    response = await test_client.post(
        "/auth/login",
        json={
            "email": registered_user["email"],
            "password": registered_user["password"],
        },
    )
    assert response.status_code == 200
    return response.json()["access_token"]


@pytest.fixture
def auth_headers(auth_token: str) -> dict[str, str]:
    """Create Authorization headers with Bearer token."""
    return {"Authorization": f"Bearer {auth_token}"}
