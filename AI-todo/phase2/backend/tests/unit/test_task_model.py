"""Unit tests for Task model validation using SQLite in-memory database."""

from datetime import datetime, timezone
from uuid import uuid4

import pytest
from pydantic import ValidationError
from sqlmodel import select

from app.models.task import Task, TaskStatus


class TestTaskModelValidation:
    """Test Task model Python/Pydantic validation."""

    # -------------------------------------------------------------------------
    # T032: Valid task creation
    # -------------------------------------------------------------------------

    def test_valid_task_creation(self) -> None:
        """Test creating a valid task with all fields."""
        user_id = uuid4()
        task = Task(
            user_id=user_id,
            title="Buy groceries",
            description="Milk, eggs, bread",
            status=TaskStatus.COMPLETED,
        )

        assert task.user_id == user_id
        assert task.title == "Buy groceries"
        assert task.description == "Milk, eggs, bread"
        assert task.status == TaskStatus.COMPLETED
        assert task.id is not None
        assert task.created_at is not None
        assert task.updated_at is not None

    def test_valid_task_minimal(self) -> None:
        """Test creating a task with only required fields."""
        user_id = uuid4()
        task = Task(
            user_id=user_id,
            title="Simple task",
        )

        assert task.user_id == user_id
        assert task.title == "Simple task"
        assert task.description is None
        assert task.status == TaskStatus.PENDING

    # -------------------------------------------------------------------------
    # T033: Empty title validation (V1)
    # -------------------------------------------------------------------------

    def test_empty_title_raises_validation_error(self) -> None:
        """Test that empty title raises ValidationError (V1)."""
        with pytest.raises(ValidationError) as exc_info:
            Task.model_validate(
                {"user_id": uuid4(), "title": ""}
            )

        assert "Title cannot be empty" in str(exc_info.value)

    # -------------------------------------------------------------------------
    # T034: Whitespace-only title validation (V1)
    # -------------------------------------------------------------------------

    def test_whitespace_title_raises_validation_error(self) -> None:
        """Test that whitespace-only title raises ValidationError (V1)."""
        with pytest.raises(ValidationError) as exc_info:
            Task.model_validate(
                {"user_id": uuid4(), "title": "   "}
            )

        assert "Title cannot be empty" in str(exc_info.value)

    def test_tabs_only_title_raises_validation_error(self) -> None:
        """Test that tabs-only title raises ValidationError."""
        with pytest.raises(ValidationError) as exc_info:
            Task.model_validate(
                {"user_id": uuid4(), "title": "\t\t\t"}
            )

        assert "Title cannot be empty" in str(exc_info.value)

    def test_newlines_only_title_raises_validation_error(self) -> None:
        """Test that newlines-only title raises ValidationError."""
        with pytest.raises(ValidationError) as exc_info:
            Task.model_validate(
                {"user_id": uuid4(), "title": "\n\n"}
            )

        assert "Title cannot be empty" in str(exc_info.value)

    # -------------------------------------------------------------------------
    # T035: Title length validation (V2)
    # -------------------------------------------------------------------------

    def test_title_max_length_valid(self) -> None:
        """Test that title at max length (200 chars) is valid."""
        task = Task(
            user_id=uuid4(),
            title="x" * 200,
        )

        assert len(task.title) == 200

    def test_title_exceeds_max_length_raises_validation_error(self) -> None:
        """Test that title exceeding 200 chars raises ValidationError (V2)."""
        with pytest.raises(ValidationError) as exc_info:
            Task.model_validate(
                {"user_id": uuid4(), "title": "x" * 201}
            )

        assert "String should have at most 200 characters" in str(exc_info.value)

    # -------------------------------------------------------------------------
    # T036: Description length validation (V5)
    # -------------------------------------------------------------------------

    def test_description_max_length_valid(self) -> None:
        """Test that description at max length (1000 chars) is valid."""
        task = Task(
            user_id=uuid4(),
            title="Test task",
            description="x" * 1000,
        )

        assert len(task.description) == 1000

    def test_description_exceeds_max_length_raises_validation_error(self) -> None:
        """Test that description exceeding 1000 chars raises ValidationError (V5)."""
        with pytest.raises(ValidationError) as exc_info:
            Task.model_validate(
                {"user_id": uuid4(), "title": "Test task", "description": "x" * 1001}
            )

        assert "String should have at most 1000 characters" in str(exc_info.value)

    # -------------------------------------------------------------------------
    # T037: Invalid status validation
    # -------------------------------------------------------------------------

    def test_invalid_status_raises_validation_error(self) -> None:
        """Test that invalid status value raises ValidationError."""
        with pytest.raises(ValidationError) as exc_info:
            Task.model_validate(
                {"user_id": uuid4(), "title": "Test task", "status": "invalid_status"}
            )

        # Check that the error mentions the valid options
        error_str = str(exc_info.value)
        assert "pending" in error_str.lower() or "completed" in error_str.lower() or "input should be" in error_str.lower()

    # -------------------------------------------------------------------------
    # T038: Default status is PENDING
    # -------------------------------------------------------------------------

    def test_default_status_is_pending(self) -> None:
        """Test that default status is PENDING."""
        task = Task(
            user_id=uuid4(),
            title="Test task",
        )

        assert task.status == TaskStatus.PENDING

    # -------------------------------------------------------------------------
    # T039: Timestamps auto-populate (M5)
    # -------------------------------------------------------------------------

    def test_timestamps_auto_populate(self) -> None:
        """Test that created_at and updated_at auto-populate on creation (M5)."""
        before = datetime.now(timezone.utc)

        task = Task(
            user_id=uuid4(),
            title="Test task",
        )

        after = datetime.now(timezone.utc)

        assert task.created_at is not None
        assert task.updated_at is not None
        assert before <= task.created_at <= after
        assert before <= task.updated_at <= after

    def test_timestamps_are_utc(self) -> None:
        """Test that timestamps are in UTC timezone."""
        task = Task(
            user_id=uuid4(),
            title="Test task",
        )

        # Timestamps should be timezone-aware UTC
        assert task.created_at.tzinfo is not None
        assert task.updated_at.tzinfo is not None

    # -------------------------------------------------------------------------
    # T040: TaskStatus enum extensibility (M7)
    # -------------------------------------------------------------------------

    def test_task_status_enum_values(self) -> None:
        """Test TaskStatus enum contains expected values."""
        assert TaskStatus.PENDING.value == "pending"
        assert TaskStatus.COMPLETED.value == "completed"

    def test_task_status_is_string_enum(self) -> None:
        """Test TaskStatus is a string enum for extensibility (M7)."""
        # TaskStatus inherits from str, so values can be used as strings
        assert isinstance(TaskStatus.PENDING, str)
        assert TaskStatus.PENDING == "pending"

    def test_task_status_all_values(self) -> None:
        """Test all TaskStatus enum members."""
        values = [status.value for status in TaskStatus]
        assert "pending" in values
        assert "completed" in values
        assert len(values) == 2  # Only pending and completed in Part 1


class TestTaskModelDatabase:
    """Test Task model database operations with SQLite."""

    # -------------------------------------------------------------------------
    # CRUD Operations with SQLite
    # -------------------------------------------------------------------------

    @pytest.mark.asyncio
    async def test_create_task_in_database(self, sqlite_session, sample_task) -> None:
        """Test creating a task in the database."""
        sqlite_session.add(sample_task)
        await sqlite_session.commit()
        await sqlite_session.refresh(sample_task)

        assert sample_task.id is not None

    @pytest.mark.asyncio
    async def test_read_task_from_database(self, sqlite_session, sample_task) -> None:
        """Test reading a task from the database."""
        sqlite_session.add(sample_task)
        await sqlite_session.commit()

        # Query the task back
        statement = select(Task).where(Task.id == sample_task.id)
        result = await sqlite_session.execute(statement)
        retrieved_task = result.scalar_one()

        assert retrieved_task.id == sample_task.id
        assert retrieved_task.title == sample_task.title
        assert retrieved_task.user_id == sample_task.user_id

    @pytest.mark.asyncio
    async def test_update_task_in_database(self, sqlite_session, sample_task) -> None:
        """Test updating a task in the database."""
        sqlite_session.add(sample_task)
        await sqlite_session.commit()

        # Update the task
        sample_task.title = "Updated Title"
        sample_task.status = TaskStatus.COMPLETED
        await sqlite_session.commit()
        await sqlite_session.refresh(sample_task)

        # Verify update
        statement = select(Task).where(Task.id == sample_task.id)
        result = await sqlite_session.execute(statement)
        retrieved_task = result.scalar_one()

        assert retrieved_task.title == "Updated Title"
        assert retrieved_task.status == TaskStatus.COMPLETED

    @pytest.mark.asyncio
    async def test_delete_task_from_database(self, sqlite_session, sample_task) -> None:
        """Test deleting a task from the database."""
        sqlite_session.add(sample_task)
        await sqlite_session.commit()

        task_id = sample_task.id

        # Delete the task
        await sqlite_session.delete(sample_task)
        await sqlite_session.commit()

        # Verify deletion
        statement = select(Task).where(Task.id == task_id)
        result = await sqlite_session.execute(statement)
        retrieved_task = result.scalar_one_or_none()

        assert retrieved_task is None

    @pytest.mark.asyncio
    async def test_user_id_uses_random_uuid(self, sample_task) -> None:
        """Test that user_id uses random UUID (T8)."""
        # The fixture generates random UUIDs
        assert sample_task.user_id is not None

        # Create another task to verify different UUIDs
        another_task = Task(
            user_id=uuid4(),
            title="Another task",
        )

        assert another_task.user_id != sample_task.user_id
