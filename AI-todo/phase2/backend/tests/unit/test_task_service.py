"""Unit tests for TaskService with ownership filtering."""

from uuid import uuid4

import pytest
from sqlmodel.ext.asyncio.session import AsyncSession

from app.models.task import TaskStatus
from app.schemas.task import TaskCreate, TaskUpdate
from app.services import task_service


class TestGetTasks:
    """T043: Test TaskService get_tasks() filtering."""

    @pytest.mark.asyncio
    async def test_get_tasks_returns_only_user_tasks(
        self, sqlite_session: AsyncSession
    ) -> None:
        """Get tasks should return only tasks for the specified user."""
        user1_id = uuid4()
        user2_id = uuid4()

        # Create tasks for user1
        await task_service.create_task(
            sqlite_session, user1_id, TaskCreate(title="User1 Task 1")
        )
        await task_service.create_task(
            sqlite_session, user1_id, TaskCreate(title="User1 Task 2")
        )

        # Create task for user2
        await task_service.create_task(
            sqlite_session, user2_id, TaskCreate(title="User2 Task")
        )

        # Get tasks for user1 (returns tuple of tasks, total_count)
        user1_tasks, total = await task_service.get_tasks(sqlite_session, user1_id)

        assert len(user1_tasks) == 2
        assert total == 2
        assert all(t.user_id == user1_id for t in user1_tasks)

    @pytest.mark.asyncio
    async def test_get_tasks_empty_for_new_user(
        self, sqlite_session: AsyncSession
    ) -> None:
        """Get tasks should return empty list for user with no tasks."""
        user_id = uuid4()

        tasks, total = await task_service.get_tasks(sqlite_session, user_id)

        assert tasks == []
        assert total == 0


class TestGetTask:
    """Test TaskService get_task() with ownership."""

    @pytest.mark.asyncio
    async def test_get_task_owned(self, sqlite_session: AsyncSession) -> None:
        """Get task should return task if owned by user."""
        user_id = uuid4()
        created = await task_service.create_task(
            sqlite_session, user_id, TaskCreate(title="My Task")
        )

        task = await task_service.get_task(sqlite_session, created.id, user_id)

        assert task is not None
        assert task.id == created.id
        assert task.title == "My Task"

    @pytest.mark.asyncio
    async def test_get_task_not_owned(self, sqlite_session: AsyncSession) -> None:
        """Get task should return None if not owned by user."""
        owner_id = uuid4()
        other_user_id = uuid4()

        created = await task_service.create_task(
            sqlite_session, owner_id, TaskCreate(title="Owner's Task")
        )

        # Try to get task as different user
        task = await task_service.get_task(sqlite_session, created.id, other_user_id)

        assert task is None

    @pytest.mark.asyncio
    async def test_get_task_nonexistent(self, sqlite_session: AsyncSession) -> None:
        """Get task should return None for non-existent task."""
        user_id = uuid4()
        fake_task_id = uuid4()

        task = await task_service.get_task(sqlite_session, fake_task_id, user_id)

        assert task is None


class TestCreateTask:
    """Test TaskService create_task()."""

    @pytest.mark.asyncio
    async def test_create_task_success(self, sqlite_session: AsyncSession) -> None:
        """Create task should set user_id and default status."""
        user_id = uuid4()
        data = TaskCreate(title="New Task", description="Task description")

        task = await task_service.create_task(sqlite_session, user_id, data)

        assert task.id is not None
        assert task.user_id == user_id
        assert task.title == "New Task"
        assert task.description == "Task description"
        assert task.status == TaskStatus.PENDING

    @pytest.mark.asyncio
    async def test_create_task_without_description(
        self, sqlite_session: AsyncSession
    ) -> None:
        """Create task without description should succeed."""
        user_id = uuid4()
        data = TaskCreate(title="Task without description")

        task = await task_service.create_task(sqlite_session, user_id, data)

        assert task.title == "Task without description"
        assert task.description is None


class TestUpdateTask:
    """Test TaskService update_task() with ownership."""

    @pytest.mark.asyncio
    async def test_update_task_owned(self, sqlite_session: AsyncSession) -> None:
        """Update task should succeed for owned task."""
        user_id = uuid4()
        created = await task_service.create_task(
            sqlite_session, user_id, TaskCreate(title="Original")
        )

        updated = await task_service.update_task(
            sqlite_session,
            created.id,
            user_id,
            TaskUpdate(title="Updated Title", status=TaskStatus.COMPLETED),
        )

        assert updated is not None
        assert updated.title == "Updated Title"
        assert updated.status == TaskStatus.COMPLETED

    @pytest.mark.asyncio
    async def test_update_task_not_owned(self, sqlite_session: AsyncSession) -> None:
        """Update task should return None for non-owned task."""
        owner_id = uuid4()
        other_user_id = uuid4()

        created = await task_service.create_task(
            sqlite_session, owner_id, TaskCreate(title="Owner's Task")
        )

        updated = await task_service.update_task(
            sqlite_session,
            created.id,
            other_user_id,
            TaskUpdate(title="Hacked Title"),
        )

        assert updated is None

    @pytest.mark.asyncio
    async def test_update_task_partial(self, sqlite_session: AsyncSession) -> None:
        """Update task with partial data should only update provided fields."""
        user_id = uuid4()
        created = await task_service.create_task(
            sqlite_session,
            user_id,
            TaskCreate(title="Original", description="Original Description"),
        )

        # Only update title
        updated = await task_service.update_task(
            sqlite_session,
            created.id,
            user_id,
            TaskUpdate(title="New Title"),
        )

        assert updated is not None
        assert updated.title == "New Title"
        assert updated.description == "Original Description"  # Unchanged


class TestDeleteTask:
    """Test TaskService delete_task() with ownership."""

    @pytest.mark.asyncio
    async def test_delete_task_owned(self, sqlite_session: AsyncSession) -> None:
        """Delete task should succeed for owned task."""
        user_id = uuid4()
        created = await task_service.create_task(
            sqlite_session, user_id, TaskCreate(title="To Delete")
        )

        deleted = await task_service.delete_task(sqlite_session, created.id, user_id)

        assert deleted is True

        # Verify task is gone
        task = await task_service.get_task(sqlite_session, created.id, user_id)
        assert task is None

    @pytest.mark.asyncio
    async def test_delete_task_not_owned(self, sqlite_session: AsyncSession) -> None:
        """Delete task should return False for non-owned task."""
        owner_id = uuid4()
        other_user_id = uuid4()

        created = await task_service.create_task(
            sqlite_session, owner_id, TaskCreate(title="Owner's Task")
        )

        deleted = await task_service.delete_task(
            sqlite_session, created.id, other_user_id
        )

        assert deleted is False

        # Verify task still exists
        task = await task_service.get_task(sqlite_session, created.id, owner_id)
        assert task is not None

    @pytest.mark.asyncio
    async def test_delete_task_nonexistent(self, sqlite_session: AsyncSession) -> None:
        """Delete task should return False for non-existent task."""
        user_id = uuid4()
        fake_task_id = uuid4()

        deleted = await task_service.delete_task(sqlite_session, fake_task_id, user_id)

        assert deleted is False
