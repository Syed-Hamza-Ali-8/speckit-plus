"""Unit tests for security functions (password hashing and JWT)."""

from datetime import timedelta
from uuid import uuid4

import pytest

from app.core.security import (
    create_access_token,
    decode_token,
    hash_password,
    verify_password,
)


class TestPasswordHashing:
    """T036: Test password hash/verify roundtrip."""

    def test_hash_password_returns_hash(self) -> None:
        """Hash password should return an Argon2id hash string."""
        password = "securepassword123"
        hashed = hash_password(password)

        assert hashed is not None
        assert hashed != password
        assert hashed.startswith("$argon2id$")

    def test_verify_password_correct(self) -> None:
        """Verify password should return True for correct password."""
        password = "securepassword123"
        hashed = hash_password(password)

        assert verify_password(password, hashed) is True

    def test_verify_password_incorrect(self) -> None:
        """Verify password should return False for incorrect password."""
        password = "securepassword123"
        hashed = hash_password(password)

        assert verify_password("wrongpassword", hashed) is False

    def test_different_passwords_different_hashes(self) -> None:
        """Different passwords should produce different hashes."""
        hash1 = hash_password("password1")
        hash2 = hash_password("password2")

        assert hash1 != hash2

    def test_same_password_different_hashes(self) -> None:
        """Same password hashed twice should produce different hashes (due to salt)."""
        password = "securepassword123"
        hash1 = hash_password(password)
        hash2 = hash_password(password)

        assert hash1 != hash2
        # But both should verify correctly
        assert verify_password(password, hash1) is True
        assert verify_password(password, hash2) is True


class TestJWTTokens:
    """T037-T039: Test JWT create/decode and error handling."""

    def test_create_access_token(self) -> None:
        """T037: Create access token should return a valid JWT string."""
        user_id = uuid4()
        email = "test@example.com"

        token = create_access_token(user_id=user_id, email=email)

        assert token is not None
        assert isinstance(token, str)
        assert len(token) > 0
        # JWT has 3 parts separated by dots
        assert len(token.split(".")) == 3

    def test_decode_token_valid(self) -> None:
        """T037: Decode valid token should return payload with claims."""
        user_id = uuid4()
        email = "test@example.com"

        token = create_access_token(user_id=user_id, email=email)
        payload = decode_token(token)

        assert payload is not None
        assert payload["sub"] == str(user_id)
        assert payload["email"] == email
        assert "exp" in payload
        assert "iat" in payload

    def test_decode_token_expired(self) -> None:
        """T038: Decode expired token should return None."""
        user_id = uuid4()
        email = "test@example.com"

        # Create token that expires immediately (negative delta)
        token = create_access_token(
            user_id=user_id,
            email=email,
            expires_delta=timedelta(seconds=-1),
        )
        payload = decode_token(token)

        assert payload is None

    def test_decode_token_invalid(self) -> None:
        """T039: Decode invalid token should return None."""
        invalid_tokens = [
            "invalid.token.here",
            "notavalidjwt",
            "",
            "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.invalid.signature",
        ]

        for invalid_token in invalid_tokens:
            payload = decode_token(invalid_token)
            assert payload is None, f"Expected None for token: {invalid_token}"

    def test_decode_token_wrong_secret(self) -> None:
        """T039: Token signed with different secret should fail to decode."""
        # This test ensures tokens from other sources are rejected
        # A token created with wrong secret would fail signature verification
        wrong_secret_token = (
            "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9."
            "eyJzdWIiOiIxMjM0NTY3ODkwIiwiZW1haWwiOiJ0ZXN0QGV4YW1wbGUuY29tIn0."
            "WrongSignatureHere"
        )

        payload = decode_token(wrong_secret_token)
        assert payload is None

    def test_create_token_with_custom_expiry(self) -> None:
        """Create token with custom expiry delta."""
        user_id = uuid4()
        email = "test@example.com"

        token = create_access_token(
            user_id=user_id,
            email=email,
            expires_delta=timedelta(hours=1),
        )
        payload = decode_token(token)

        assert payload is not None
        assert payload["sub"] == str(user_id)
