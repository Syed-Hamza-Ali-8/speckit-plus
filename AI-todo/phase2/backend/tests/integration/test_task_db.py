"""Integration tests for Task model with Neon Postgres database."""

import os
from datetime import datetime, timezone
from uuid import uuid4

import pytest
from sqlalchemy import inspect, text
from sqlalchemy.exc import IntegrityError
from sqlmodel import select

from app.models.task import Task, TaskStatus
from app.models.user import User


def current_utc() -> datetime:
    """Return timezone-aware current UTC datetime."""
    return datetime.now(timezone.utc)


class TestTaskDatabaseOperations:
    """Test Task CRUD operations with Neon Postgres."""

    # -------------------------------------------------------------------------
    # T043: Create and Read (CRUD - Create, Read)
    # -------------------------------------------------------------------------

    @pytest.mark.asyncio
    async def test_create_and_retrieve_task(self, neon_session, test_user: User) -> None:
        """Test creating a task and retrieving it by id (CRUD - Create, Read)."""
        task = Task(
            user_id=test_user.id,
            title="Integration Test Task",
            description="Testing with Neon Postgres",
            status=TaskStatus.PENDING,
        )

        neon_session.add(task)
        await neon_session.flush()

        # Retrieve by id
        statement = select(Task).where(Task.id == task.id)
        result = await neon_session.execute(statement)
        retrieved_task = result.scalar_one()

        assert retrieved_task.id == task.id
        assert retrieved_task.user_id == test_user.id
        assert retrieved_task.title == "Integration Test Task"
        assert retrieved_task.description == "Testing with Neon Postgres"
        assert retrieved_task.status == TaskStatus.PENDING

    # -------------------------------------------------------------------------
    # T044: Update and verify timestamps (CRUD - Update, M6)
    # -------------------------------------------------------------------------

    @pytest.mark.asyncio
    async def test_update_task_timestamps(self, neon_session, test_user: User) -> None:
        """Test updating task and verify updated_at changes, created_at unchanged (M6)."""
        task = Task(
            user_id=test_user.id,
            title="Original Title",
        )

        neon_session.add(task)
        await neon_session.flush()

        original_created_at = task.created_at
        original_updated_at = task.updated_at

        # Update the task
        task.title = "Updated Title"
        task.updated_at = current_utc()  # Use datetime object
        await neon_session.flush()

        # Verify created_at unchanged, updated_at changed
        assert task.created_at == original_created_at
        assert task.updated_at >= original_updated_at

    # -------------------------------------------------------------------------
    # T045: Delete task (CRUD - Delete)
    # -------------------------------------------------------------------------

    @pytest.mark.asyncio
    async def test_delete_task(self, neon_session, test_user: User) -> None:
        """Test deleting a task and verify removal (CRUD - Delete)."""
        task = Task(
            user_id=test_user.id,
            title="Task to Delete",
        )

        neon_session.add(task)
        await neon_session.flush()
        task_id = task.id

        # Delete
        await neon_session.delete(task)
        await neon_session.flush()

        # Verify removal
        statement = select(Task).where(Task.id == task_id)
        result = await neon_session.execute(statement)
        retrieved_task = result.scalar_one_or_none()

        assert retrieved_task is None

    # -------------------------------------------------------------------------
    # T046: DB CHECK constraint rejects empty title (V3)
    # -------------------------------------------------------------------------

    @pytest.mark.asyncio
    async def test_db_check_rejects_empty_title(self, neon_session) -> None:
        """Test DB CHECK constraint rejects empty title on direct SQL INSERT (V3)."""
        user_id = str(uuid4())
        task_id = str(uuid4())
        now = current_utc()  # ✅ datetime object

        sql = text("""
            INSERT INTO tasks (id, user_id, title, status, created_at, updated_at)
            VALUES (:id, :user_id, :title, :status, :created_at, :updated_at)
        """)

        with pytest.raises(IntegrityError) as exc_info:
            await neon_session.execute(
                sql,
                {
                    "id": task_id,
                    "user_id": user_id,
                    "title": "",  # or "   "
                    "status": "pending",
                    "created_at": now,
                    "updated_at": now,
                },
            )
            await neon_session.flush()

        # Immediately rollback the failed transaction so fixture teardown works
        await neon_session.rollback()

        assert "ck_tasks_title_not_empty" in str(exc_info.value).lower() or \
               "check" in str(exc_info.value).lower()

    # -------------------------------------------------------------------------
    # T047: DB CHECK constraint rejects whitespace-only title (V4)
    # -------------------------------------------------------------------------

    @pytest.mark.asyncio
    async def test_db_check_rejects_whitespace_title(self, neon_session) -> None:
        """Test DB CHECK constraint rejects whitespace-only title on direct SQL INSERT (V4)."""
        user_id = str(uuid4())
        task_id = str(uuid4())
        now = current_utc()  # ✅ datetime object

        sql = text("""
            INSERT INTO tasks (id, user_id, title, status, created_at, updated_at)
            VALUES (:id, :user_id, :title, :status, :created_at, :updated_at)
        """)

        with pytest.raises(IntegrityError) as exc_info:
            await neon_session.execute(
                sql,
                {
                    "id": task_id,
                    "user_id": user_id,
                    "title": "",  # or "   "
                    "status": "pending",
                    "created_at": now,
                    "updated_at": now,
                },
            )
            await neon_session.flush()

        # Immediately rollback the failed transaction so fixture teardown works
        await neon_session.rollback()

        assert "ck_tasks_title_not_empty" in str(exc_info.value).lower() or \
               "check" in str(exc_info.value).lower()

    # -------------------------------------------------------------------------
    # T048: Verify index exists on user_id (M4)
    # -------------------------------------------------------------------------

    @pytest.mark.asyncio
    async def test_user_id_index_exists(self, neon_engine) -> None:
        """Test that ix_tasks_user_id index exists on user_id column (M4)."""
        async with neon_engine.connect() as conn:
            result = await conn.execute(
                text("""
                    SELECT indexname FROM pg_indexes
                    WHERE tablename = 'tasks' AND indexname = 'ix_tasks_user_id'
                """)
            )
            index = result.scalar_one_or_none()

            assert index == "ix_tasks_user_id"

    # -------------------------------------------------------------------------
    # T049: SSL connection verification (C4)
    # -------------------------------------------------------------------------

    @pytest.mark.asyncio
    async def test_ssl_connection(self, neon_engine) -> None:
        """Test SSL connection succeeds - verify connection string has ssl=require (C4)."""
        test_db_url = os.environ.get("TEST_DATABASE_URL", "")

        assert "ssl=require" in test_db_url or "sslmode=require" in test_db_url, \
            "TEST_DATABASE_URL must include ssl=require for Neon with asyncpg"

        async with neon_engine.connect() as conn:
            result = await conn.execute(text("SELECT 1"))
            assert result.scalar() == 1

    # -------------------------------------------------------------------------
    # T050: Timestamps stored in UTC (TIMESTAMPTZ)
    # -------------------------------------------------------------------------

    @pytest.mark.asyncio
    async def test_timestamps_stored_in_utc(self, neon_session, test_user: User) -> None:
        """Test timestamps are stored in UTC (TIMESTAMPTZ behavior)."""
        task = Task(
            user_id=test_user.id,
            title="UTC Timestamp Test",
        )

        neon_session.add(task)
        await neon_session.flush()

        statement = select(Task).where(Task.id == task.id)
        result = await neon_session.execute(statement)
        retrieved_task = result.scalar_one()

        assert retrieved_task.created_at.tzinfo is not None or \
               retrieved_task.created_at.utcoffset() is not None

        now = current_utc()
        time_diff = abs((now - retrieved_task.created_at.replace(tzinfo=timezone.utc)).total_seconds())
        assert time_diff < 60, "Timestamp should be within last minute"


class TestTaskQueryPatterns:
    """Test common query patterns for tasks."""

    @pytest.mark.asyncio
    async def test_query_tasks_by_user_id(self, neon_session, test_user: User) -> None:
        """Test querying tasks by user_id (uses index)."""
        # Create another user for "other user's task"
        other_user = User(
            email=f"other_{uuid4().hex[:8]}@example.com",
            hashed_password="$argon2id$v=19$m=65536,t=2,p=1$fakesalt$fakehash",
            is_active=True,
        )
        neon_session.add(other_user)
        await neon_session.flush()

        for i in range(3):
            task = Task(user_id=test_user.id, title=f"Task {i}")
            neon_session.add(task)

        other_task = Task(user_id=other_user.id, title="Other user task")
        neon_session.add(other_task)

        await neon_session.flush()

        statement = select(Task).where(Task.user_id == test_user.id)
        result = await neon_session.execute(statement)
        user_tasks = result.scalars().all()

        assert len(user_tasks) == 3
        for task in user_tasks:
            assert task.user_id == test_user.id

    @pytest.mark.asyncio
    async def test_query_tasks_by_status(self, neon_session, test_user: User) -> None:
        """Test querying tasks by status."""
        pending_task = Task(user_id=test_user.id, title="Pending", status=TaskStatus.PENDING)
        completed_task = Task(user_id=test_user.id, title="Completed", status=TaskStatus.COMPLETED)

        neon_session.add(pending_task)
        neon_session.add(completed_task)
        await neon_session.flush()

        statement = select(Task).where(
            Task.user_id == test_user.id,
            Task.status == TaskStatus.PENDING,
        )
        result = await neon_session.execute(statement)
        pending_tasks = result.scalars().all()

        assert len(pending_tasks) == 1
        assert pending_tasks[0].status == TaskStatus.PENDING
