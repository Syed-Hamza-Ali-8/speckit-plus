"""Unit tests for auth service functions."""

import pytest
from sqlmodel.ext.asyncio.session import AsyncSession

from app.schemas.auth import UserCreate
from app.services import auth_service


class TestRegisterUser:
    """T041: Test auth_service.register_user()."""

    @pytest.mark.asyncio
    async def test_register_user_success(self, sqlite_session: AsyncSession) -> None:
        """Register user should create a new user with hashed password."""
        data = UserCreate(email="newuser@example.com", password="securepassword123")

        user = await auth_service.register_user(sqlite_session, data)

        assert user is not None
        assert user.email == "newuser@example.com"
        assert user.hashed_password != "securepassword123"  # Password should be hashed
        assert user.hashed_password.startswith("$argon2id$")
        assert user.is_active is True
        assert user.id is not None

    @pytest.mark.asyncio
    async def test_register_user_email_normalized(
        self, sqlite_session: AsyncSession
    ) -> None:
        """Register user should normalize email to lowercase."""
        data = UserCreate(email="USER@EXAMPLE.COM", password="securepassword123")

        user = await auth_service.register_user(sqlite_session, data)

        assert user is not None
        assert user.email == "user@example.com"

    @pytest.mark.asyncio
    async def test_register_user_duplicate_email(
        self, sqlite_session: AsyncSession
    ) -> None:
        """Register user should return None for duplicate email."""
        data = UserCreate(email="duplicate@example.com", password="password123")

        # First registration should succeed
        user1 = await auth_service.register_user(sqlite_session, data)
        assert user1 is not None

        # Second registration with same email should fail
        user2 = await auth_service.register_user(sqlite_session, data)
        assert user2 is None

    @pytest.mark.asyncio
    async def test_register_user_duplicate_email_case_insensitive(
        self, sqlite_session: AsyncSession
    ) -> None:
        """Duplicate email check should be case-insensitive."""
        data1 = UserCreate(email="test@example.com", password="password123")
        data2 = UserCreate(email="TEST@EXAMPLE.COM", password="password456")

        user1 = await auth_service.register_user(sqlite_session, data1)
        assert user1 is not None

        user2 = await auth_service.register_user(sqlite_session, data2)
        assert user2 is None


class TestAuthenticateUser:
    """T042: Test auth_service.authenticate_user()."""

    @pytest.mark.asyncio
    async def test_authenticate_user_success(
        self, sqlite_session: AsyncSession
    ) -> None:
        """Authenticate user should return user for valid credentials."""
        # First register a user
        data = UserCreate(email="auth@example.com", password="correctpassword")
        await auth_service.register_user(sqlite_session, data)

        # Then authenticate
        user = await auth_service.authenticate_user(
            sqlite_session, "auth@example.com", "correctpassword"
        )

        assert user is not None
        assert user.email == "auth@example.com"

    @pytest.mark.asyncio
    async def test_authenticate_user_wrong_password(
        self, sqlite_session: AsyncSession
    ) -> None:
        """Authenticate user should return None for wrong password."""
        data = UserCreate(email="wrongpw@example.com", password="correctpassword")
        await auth_service.register_user(sqlite_session, data)

        user = await auth_service.authenticate_user(
            sqlite_session, "wrongpw@example.com", "wrongpassword"
        )

        assert user is None

    @pytest.mark.asyncio
    async def test_authenticate_user_nonexistent_email(
        self, sqlite_session: AsyncSession
    ) -> None:
        """Authenticate user should return None for non-existent email."""
        user = await auth_service.authenticate_user(
            sqlite_session, "nonexistent@example.com", "anypassword"
        )

        assert user is None

    @pytest.mark.asyncio
    async def test_authenticate_user_email_case_insensitive(
        self, sqlite_session: AsyncSession
    ) -> None:
        """Authenticate should work with different email case."""
        data = UserCreate(email="casetest@example.com", password="password123")
        await auth_service.register_user(sqlite_session, data)

        # Try to authenticate with uppercase email
        user = await auth_service.authenticate_user(
            sqlite_session, "CASETEST@EXAMPLE.COM", "password123"
        )

        assert user is not None
        assert user.email == "casetest@example.com"


class TestGetUserByEmail:
    """Test auth_service.get_user_by_email()."""

    @pytest.mark.asyncio
    async def test_get_user_by_email_found(self, sqlite_session: AsyncSession) -> None:
        """Get user by email should return user if exists."""
        data = UserCreate(email="findme@example.com", password="password123")
        await auth_service.register_user(sqlite_session, data)

        user = await auth_service.get_user_by_email(sqlite_session, "findme@example.com")

        assert user is not None
        assert user.email == "findme@example.com"

    @pytest.mark.asyncio
    async def test_get_user_by_email_not_found(
        self, sqlite_session: AsyncSession
    ) -> None:
        """Get user by email should return None if not exists."""
        user = await auth_service.get_user_by_email(
            sqlite_session, "notfound@example.com"
        )

        assert user is None


class TestGetUserById:
    """Test auth_service.get_user_by_id()."""

    @pytest.mark.asyncio
    async def test_get_user_by_id_found(self, sqlite_session: AsyncSession) -> None:
        """Get user by ID should return user if exists."""
        data = UserCreate(email="byid@example.com", password="password123")
        created_user = await auth_service.register_user(sqlite_session, data)
        assert created_user is not None

        user = await auth_service.get_user_by_id(sqlite_session, created_user.id)

        assert user is not None
        assert user.id == created_user.id
        assert user.email == "byid@example.com"
