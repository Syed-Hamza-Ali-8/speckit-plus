"""Task service with user ownership filtering.

All operations enforce user_id filtering to ensure users can only
access their own tasks.
"""

from uuid import UUID

from sqlmodel import select
from sqlmodel.ext.asyncio.session import AsyncSession

from app.models.task import Task, TaskStatus
from app.schemas.task import TaskCreate, TaskUpdate


async def get_tasks(db: AsyncSession, user_id: UUID) -> list[Task]:
    """Get all tasks for a specific user.

    Args:
        db: Async database session.
        user_id: Owner's user ID.

    Returns:
        List of tasks owned by the user.
    """
    statement = select(Task).where(Task.user_id == user_id).order_by(Task.created_at.desc())
    result = await db.exec(statement)
    return list(result.all())


async def get_task(db: AsyncSession, task_id: UUID, user_id: UUID) -> Task | None:
    """Get a specific task if owned by the user.

    Args:
        db: Async database session.
        task_id: Task's unique identifier.
        user_id: Owner's user ID.

    Returns:
        Task if found and owned by user, None otherwise.
    """
    statement = select(Task).where(Task.id == task_id, Task.user_id == user_id)
    result = await db.exec(statement)
    return result.first()


async def create_task(db: AsyncSession, user_id: UUID, data: TaskCreate) -> Task:
    """Create a new task for a user.

    Args:
        db: Async database session.
        user_id: Owner's user ID.
        data: Task creation data.

    Returns:
        Newly created task.
    """
    task = Task(
        user_id=user_id,
        title=data.title,
        description=data.description,
        status=TaskStatus.PENDING,
    )
    db.add(task)
    await db.commit()
    await db.refresh(task)
    return task


async def update_task(
    db: AsyncSession,
    task_id: UUID,
    user_id: UUID,
    data: TaskUpdate,
) -> Task | None:
    """Update a task if owned by the user.

    Args:
        db: Async database session.
        task_id: Task's unique identifier.
        user_id: Owner's user ID.
        data: Task update data (partial update supported).

    Returns:
        Updated task if found and owned by user, None otherwise.
    """
    task = await get_task(db, task_id, user_id)
    if task is None:
        return None

    # Apply partial update - only update non-None fields
    update_data = data.model_dump(exclude_unset=True)
    for field, value in update_data.items():
        setattr(task, field, value)

    db.add(task)
    await db.commit()
    await db.refresh(task)
    return task


async def delete_task(db: AsyncSession, task_id: UUID, user_id: UUID) -> bool:
    """Delete a task if owned by the user.

    Args:
        db: Async database session.
        task_id: Task's unique identifier.
        user_id: Owner's user ID.

    Returns:
        True if task was deleted, False if not found or not owned.
    """
    task = await get_task(db, task_id, user_id)
    if task is None:
        return False

    await db.delete(task)
    await db.commit()
    return True
