"""Task service with user ownership filtering.

All operations enforce user_id filtering to ensure users can only
access their own tasks.
"""

from datetime import date
from uuid import UUID

from sqlalchemy import func
from sqlmodel import select
from sqlmodel.ext.asyncio.session import AsyncSession

from app.models.task import Task, TaskStatus
from app.schemas.task import TaskCreate, TaskUpdate


# Allowed sort fields and directions for validation
ALLOWED_SORT_FIELDS = {"created_at", "updated_at", "title", "status"}
ALLOWED_SORT_DIRECTIONS = {"asc", "desc"}


def parse_sort(sort: str) -> tuple[str, str]:
    """Parse and validate sort parameter string.

    Args:
        sort: Sort string in format "field:direction" (e.g., "created_at:desc")

    Returns:
        Tuple of (field, direction)

    Raises:
        ValueError: If sort format is invalid or field/direction not allowed
    """
    parts = sort.split(":")
    if len(parts) != 2:
        raise ValueError("Sort must be in format 'field:direction'")

    field, direction = parts
    if field not in ALLOWED_SORT_FIELDS:
        raise ValueError(f"Invalid sort field: {field}. Allowed: {', '.join(ALLOWED_SORT_FIELDS)}")
    if direction not in ALLOWED_SORT_DIRECTIONS:
        raise ValueError(f"Invalid sort direction: {direction}. Allowed: asc, desc")

    return field, direction


async def get_tasks(
    db: AsyncSession,
    user_id: UUID,
    status: TaskStatus | None = None,
    created_after: date | None = None,
    created_before: date | None = None,
    sort_field: str = "created_at",
    sort_direction: str = "desc",
    limit: int = 20,
    offset: int = 0,
) -> tuple[list[Task], int]:
    """Get paginated tasks for a specific user with filtering and sorting.

    Args:
        db: Async database session.
        user_id: Owner's user ID.
        status: Optional filter by task status.
        created_after: Optional filter for tasks created after this date.
        created_before: Optional filter for tasks created before this date.
        sort_field: Field to sort by (default: created_at).
        sort_direction: Sort direction - "asc" or "desc" (default: desc).
        limit: Maximum number of tasks to return (default: 20).
        offset: Number of tasks to skip (default: 0).

    Returns:
        Tuple of (list of tasks, total count before pagination).
    """
    # Build base query with user filter
    base_query = select(Task).where(Task.user_id == user_id)

    # Apply optional filters
    if status is not None:
        base_query = base_query.where(Task.status == status)
    if created_after is not None:
        base_query = base_query.where(Task.created_at >= created_after)
    if created_before is not None:
        base_query = base_query.where(Task.created_at <= created_before)

    # Get total count before pagination
    count_query = select(func.count()).select_from(base_query.subquery())
    total = await db.scalar(count_query) or 0

    # Apply sorting
    sort_column = getattr(Task, sort_field)
    if sort_direction == "desc":
        base_query = base_query.order_by(sort_column.desc())
    else:
        base_query = base_query.order_by(sort_column.asc())

    # Apply pagination
    base_query = base_query.offset(offset).limit(limit)

    # Execute query
    result = await db.exec(base_query)
    tasks = list(result.all())

    return tasks, total


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
