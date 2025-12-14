"""Integration tests for authentication endpoints."""

from uuid import uuid4

import pytest
from httpx import AsyncClient


class TestRegistration:
    """T045-T046: Test registration endpoint."""

    @pytest.mark.asyncio
    async def test_valid_registration_returns_201(
        self, test_client: AsyncClient
    ) -> None:
        """TC-01: Valid registration should return 201 and user data."""
        email = f"newuser_{uuid4().hex[:8]}@example.com"
        response = await test_client.post(
            "/auth/register",
            json={"email": email, "password": "securepassword123"},
        )

        assert response.status_code == 201
        data = response.json()
        assert data["email"] == email
        assert data["is_active"] is True
        assert "id" in data
        assert "created_at" in data
        # Password should not be in response
        assert "password" not in data
        assert "hashed_password" not in data

    @pytest.mark.asyncio
    async def test_duplicate_email_returns_409(
        self, test_client: AsyncClient
    ) -> None:
        """TC-02: Duplicate email should return 409 Conflict."""
        email = f"duplicate_{uuid4().hex[:8]}@example.com"

        # First registration
        response1 = await test_client.post(
            "/auth/register",
            json={"email": email, "password": "password123"},
        )
        assert response1.status_code == 201

        # Second registration with same email
        response2 = await test_client.post(
            "/auth/register",
            json={"email": email, "password": "differentpassword"},
        )
        assert response2.status_code == 409
        assert "already registered" in response2.json()["detail"].lower()

    @pytest.mark.asyncio
    async def test_invalid_email_returns_422(self, test_client: AsyncClient) -> None:
        """Invalid email format should return 422 Validation Error."""
        response = await test_client.post(
            "/auth/register",
            json={"email": "not-an-email", "password": "password123"},
        )

        assert response.status_code == 422

    @pytest.mark.asyncio
    async def test_short_password_returns_422(self, test_client: AsyncClient) -> None:
        """Password shorter than 8 characters should return 422."""
        response = await test_client.post(
            "/auth/register",
            json={"email": "test@example.com", "password": "short"},
        )

        assert response.status_code == 422


class TestLogin:
    """T047-T048: Test login endpoint."""

    @pytest.mark.asyncio
    async def test_valid_login_returns_jwt(
        self, test_client: AsyncClient, registered_user: dict
    ) -> None:
        """TC-06: Valid login should return 200 and JWT token."""
        response = await test_client.post(
            "/auth/login",
            json={
                "email": registered_user["email"],
                "password": registered_user["password"],
            },
        )

        assert response.status_code == 200
        data = response.json()
        assert "access_token" in data
        assert data["token_type"] == "bearer"
        # Token should be a valid JWT format (3 parts)
        assert len(data["access_token"].split(".")) == 3

    @pytest.mark.asyncio
    async def test_wrong_password_returns_401(
        self, test_client: AsyncClient, registered_user: dict
    ) -> None:
        """TC-07: Wrong password should return 401 Unauthorized."""
        response = await test_client.post(
            "/auth/login",
            json={
                "email": registered_user["email"],
                "password": "wrongpassword",
            },
        )

        assert response.status_code == 401
        assert "invalid" in response.json()["detail"].lower()

    @pytest.mark.asyncio
    async def test_nonexistent_email_returns_401(
        self, test_client: AsyncClient
    ) -> None:
        """Non-existent email should return 401."""
        response = await test_client.post(
            "/auth/login",
            json={
                "email": "nonexistent@example.com",
                "password": "anypassword",
            },
        )

        assert response.status_code == 401


class TestProfile:
    """T049-T050: Test /auth/me endpoint."""

    @pytest.mark.asyncio
    async def test_valid_token_returns_user(
        self, test_client: AsyncClient, registered_user: dict, auth_headers: dict
    ) -> None:
        """TC-10: Valid token should return user profile."""
        response = await test_client.get("/auth/me", headers=auth_headers)

        assert response.status_code == 200
        data = response.json()
        assert data["email"] == registered_user["email"]
        assert data["id"] == registered_user["id"]
        assert data["is_active"] is True
        assert "created_at" in data

    @pytest.mark.asyncio
    async def test_no_token_returns_401(self, test_client: AsyncClient) -> None:
        """TC-11: No token should return 401 Unauthorized."""
        response = await test_client.get("/auth/me")

        assert response.status_code == 401

    @pytest.mark.asyncio
    async def test_invalid_token_returns_401(self, test_client: AsyncClient) -> None:
        """Invalid token should return 401."""
        response = await test_client.get(
            "/auth/me",
            headers={"Authorization": "Bearer invalid.token.here"},
        )

        assert response.status_code == 401

    @pytest.mark.asyncio
    async def test_malformed_auth_header_returns_401(
        self, test_client: AsyncClient
    ) -> None:
        """Malformed Authorization header should return 401."""
        response = await test_client.get(
            "/auth/me",
            headers={"Authorization": "NotBearer sometoken"},
        )

        assert response.status_code == 401


class TestRateLimiting:
    """T053-T054: Test rate limiting on auth endpoints."""

    @pytest.mark.asyncio
    async def test_register_rate_limit_returns_429(
        self, test_client_with_rate_limit: AsyncClient
    ) -> None:
        """TC-19: Exceeding register rate limit should return 429."""
        # Make 6 requests (limit is 5/minute)
        responses = []
        for i in range(6):
            response = await test_client_with_rate_limit.post(
                "/auth/register",
                json={
                    "email": f"ratelimit_{uuid4().hex[:8]}@example.com",
                    "password": "password123",
                },
            )
            responses.append(response.status_code)

        # At least one should be 429
        assert 429 in responses, f"Expected 429 in responses: {responses}"

    @pytest.mark.asyncio
    async def test_login_rate_limit_returns_429(
        self, test_client_with_rate_limit: AsyncClient
    ) -> None:
        """TC-20: Exceeding login rate limit should return 429."""
        # Make 11 requests (limit is 10/minute)
        responses = []
        for i in range(11):
            response = await test_client_with_rate_limit.post(
                "/auth/login",
                json={
                    "email": "ratelimit@example.com",
                    "password": "anypassword",
                },
            )
            responses.append(response.status_code)

        # At least one should be 429
        assert 429 in responses, f"Expected 429 in responses: {responses}"
