"""Unit tests for User model validation."""

import pytest
from pydantic import ValidationError

from app.models.user import User


class TestUserModel:
    """T040: Test User model validation."""

    def test_create_valid_user(self) -> None:
        """Valid user creation should succeed."""
        user = User(
            email="test@example.com",
            hashed_password="$argon2id$v=19$m=65536,t=2,p=1$salt$hash",
            is_active=True,
        )

        assert user.email == "test@example.com"
        assert user.hashed_password.startswith("$argon2id$")
        assert user.is_active is True
        assert user.id is not None
        assert user.created_at is not None

    def test_email_normalization_lowercase(self) -> None:
        """Email should be normalized to lowercase."""
        user = User(
            email="TEST@EXAMPLE.COM",
            hashed_password="hash",
        )

        assert user.email == "test@example.com"

    def test_email_normalization_whitespace(self) -> None:
        """Email should have whitespace stripped."""
        user = User(
            email="  test@example.com  ",
            hashed_password="hash",
        )

        assert user.email == "test@example.com"

    def test_email_empty_raises_error(self) -> None:
        """Empty email should raise validation error."""
        from pydantic import ValidationError
        with pytest.raises((ValueError, ValidationError)):
            User(
                email="",
                hashed_password="hash",
            )

    def test_email_whitespace_only_raises_error(self) -> None:
        """Whitespace-only email should raise validation error."""
        from pydantic import ValidationError
        with pytest.raises((ValueError, ValidationError)):
            User(
                email="   ",
                hashed_password="hash",
            )

    def test_default_is_active_true(self) -> None:
        """Default is_active should be True."""
        user = User(
            email="test@example.com",
            hashed_password="hash",
        )

        assert user.is_active is True

    def test_user_id_auto_generated(self) -> None:
        """User ID should be auto-generated UUID."""
        user = User(
            email="test@example.com",
            hashed_password="hash",
        )

        assert user.id is not None
        # Check it's a valid UUID by accessing its hex property
        assert len(str(user.id)) == 36  # UUID string format

    def test_created_at_auto_generated(self) -> None:
        """Created at should be auto-generated."""
        user = User(
            email="test@example.com",
            hashed_password="hash",
        )

        assert user.created_at is not None

    def test_two_users_different_ids(self) -> None:
        """Two users should have different IDs."""
        user1 = User(email="user1@example.com", hashed_password="hash1")
        user2 = User(email="user2@example.com", hashed_password="hash2")

        assert user1.id != user2.id
