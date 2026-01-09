"""Task service with user ownership filtering.

All operations enforce user_id filtering to ensure users can only
access their own tasks.
"""

from datetime import date, datetime, timedelta
from uuid import UUID
from typing import List, Optional

from sqlalchemy import func, or_
from sqlmodel import select
from sqlmodel.ext.asyncio.session import AsyncSession

from app.models.task import Task, TaskStatus, PriorityLevel
from app.models.recurring_task_pattern import RecurringTaskPattern, RecurrencePattern
from app.schemas.task import TaskCreate, TaskUpdate


# Allowed sort fields and directions for validation
ALLOWED_SORT_FIELDS = {"created_at", "updated_at", "title", "status", "priority", "due_date"}
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
    priority: PriorityLevel | None = None,
    tags: List[str] | None = None,
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
        priority: Optional filter by priority level.
        tags: Optional filter by tags.
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
    if priority is not None:
        base_query = base_query.where(Task.priority == priority)
    if tags:
        # Filter tasks that contain all specified tags
        for tag in tags:
            base_query = base_query.where(func.json_contains(Task.tags, f'"{tag}"'))

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
        due_date=data.due_date,
        status=TaskStatus.PENDING,
        priority=data.priority or PriorityLevel.MEDIUM,
        tags=data.tags or [],
        is_recurring=data.is_recurring,
        recurring_pattern_id=data.recurring_pattern_id,
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


# Phase V: Advanced features service functions
async def set_task_priority(
    db: AsyncSession,
    task_id: UUID,
    user_id: UUID,
    priority: PriorityLevel,
) -> Task | None:
    """Set the priority of a task if owned by the user.

    Args:
        db: Async database session.
        task_id: Task's unique identifier.
        user_id: Owner's user ID.
        priority: New priority level.

    Returns:
        Updated task if found and owned by user, None otherwise.
    """
    task = await get_task(db, task_id, user_id)
    if task is None:
        return None

    task.priority = priority
    db.add(task)
    await db.commit()
    await db.refresh(task)
    return task


async def add_task_tags(
    db: AsyncSession,
    task_id: UUID,
    user_id: UUID,
    tags: List[str],
) -> Task | None:
    """Add tags to a task if owned by the user.

    Args:
        db: Async database session.
        task_id: Task's unique identifier.
        user_id: Owner's user ID.
        tags: List of tags to add.

    Returns:
        Updated task if found and owned by user, None otherwise.
    """
    task = await get_task(db, task_id, user_id)
    if task is None:
        return None

    # Add new tags, avoiding duplicates
    existing_tags = set(task.tags)
    new_tags = set(tags)
    all_tags = list(existing_tags.union(new_tags))

    task.tags = all_tags
    db.add(task)
    await db.commit()
    await db.refresh(task)
    return task


async def remove_task_tags(
    db: AsyncSession,
    task_id: UUID,
    user_id: UUID,
    tags: List[str],
) -> Task | None:
    """Remove tags from a task if owned by the user.

    Args:
        db: Async database session.
        task_id: Task's unique identifier.
        user_id: Owner's user ID.
        tags: List of tags to remove.

    Returns:
        Updated task if found and owned by user, None otherwise.
    """
    task = await get_task(db, task_id, user_id)
    if task is None:
        return None

    # Remove specified tags
    current_tags = set(task.tags)
    tags_to_remove = set(tags)
    remaining_tags = list(current_tags - tags_to_remove)

    task.tags = remaining_tags
    db.add(task)
    await db.commit()
    await db.refresh(task)
    return task


async def search_tasks(
    db: AsyncSession,
    user_id: UUID,
    query: str,
    status: str | None = None,
    priority: PriorityLevel | None = None,
    tags: List[str] | None = None,
    due_before: date | None = None,
    due_after: date | None = None,
    sort_by: str = "created_at",
    order: str = "desc",
    page: int = 1,
    per_page: int = 20,
) -> tuple[list[Task], int]:
    """Search tasks with various filters and sorting options.

    Args:
        db: Async database session.
        user_id: Owner's user ID.
        query: Search query string to match in title or description.
        status: Optional filter by status ("all", "pending", "completed").
        priority: Optional filter by priority level.
        tags: Optional filter by tags.
        due_before: Optional filter for tasks due before this date.
        due_after: Optional filter for tasks due after this date.
        sort_by: Field to sort by (default: created_at).
        order: Sort order ("asc" or "desc", default: desc).
        page: Page number for pagination (default: 1).
        per_page: Number of items per page (default: 20).

    Returns:
        Tuple of (list of tasks, total count before pagination).
    """
    # Build base query with user filter
    base_query = select(Task).where(Task.user_id == user_id)

    # Apply text search
    if query:
        base_query = base_query.where(
            or_(
                Task.title.ilike(f"%{query}%"),
                Task.description.ilike(f"%{query}%")
            )
        )

    # Apply status filter
    if status and status != "all":
        if status == "pending":
            base_query = base_query.where(Task.status == TaskStatus.PENDING)
        elif status == "completed":
            base_query = base_query.where(Task.status == TaskStatus.COMPLETED)

    # Apply priority filter
    if priority:
        base_query = base_query.where(Task.priority == priority)

    # Apply tags filter
    if tags:
        # Filter tasks that contain all specified tags
        for tag in tags:
            base_query = base_query.where(func.json_contains(Task.tags, f'"{tag}"'))

    # Apply due date filters
    if due_before:
        base_query = base_query.where(Task.due_date <= due_before)
    if due_after:
        base_query = base_query.where(Task.due_date >= due_after)

    # Get total count before pagination
    count_query = select(func.count()).select_from(base_query.subquery())
    total = await db.scalar(count_query) or 0

    # Apply sorting
    if sort_by == "created_at":
        sort_column = Task.created_at
    elif sort_by == "title":
        sort_column = Task.title
    elif sort_by == "due_date":
        sort_column = Task.due_date
    elif sort_by == "priority":
        sort_column = Task.priority
    else:
        sort_column = Task.created_at

    if order == "desc":
        base_query = base_query.order_by(sort_column.desc())
    else:
        base_query = base_query.order_by(sort_column.asc())

    # Apply pagination
    offset = (page - 1) * per_page
    base_query = base_query.offset(offset).limit(per_page)

    # Execute query
    result = await db.exec(base_query)
    tasks = list(result.all())

    return tasks, total


async def create_recurring_task_pattern(
    db: AsyncSession,
    user_id: UUID,
    base_task_title: str,
    base_task_description: str | None,
    pattern_type: RecurrencePattern,
    interval: int = 1,
    start_date: date | None = None,
    end_date: date | None = None,
    weekdays: List[int] | None = None,
    days_of_month: List[int] | None = None,
) -> RecurringTaskPattern:
    """Create a recurring task pattern.

    Args:
        db: Async database session.
        user_id: Owner's user ID.
        base_task_title: Title for the base recurring task.
        base_task_description: Description for the base recurring task.
        pattern_type: Type of recurrence pattern.
        interval: Interval between occurrences (default: 1).
        start_date: Start date for the recurring pattern (default: today).
        end_date: Optional end date for the recurring pattern.
        weekdays: Optional list of weekdays for weekly patterns.
        days_of_month: Optional list of days of month for monthly patterns.

    Returns:
        Newly created recurring task pattern.
    """
    if start_date is None:
        start_date = datetime.now().date()

    if weekdays is None:
        weekdays = []

    if days_of_month is None:
        days_of_month = []

    pattern = RecurringTaskPattern(
        user_id=user_id,
        base_task_title=base_task_title,
        base_task_description=base_task_description,
        pattern_type=pattern_type,
        interval=interval,
        start_date=start_date,
        end_date=end_date,
        weekdays=weekdays,
        days_of_month=days_of_month,
    )
    db.add(pattern)
    await db.commit()
    await db.refresh(pattern)
    return pattern


async def get_recurring_task_pattern(
    db: AsyncSession,
    pattern_id: UUID,
    user_id: UUID
) -> RecurringTaskPattern | None:
    """Get a specific recurring task pattern if owned by the user.

    Args:
        db: Async database session.
        pattern_id: Pattern's unique identifier.
        user_id: Owner's user ID.

    Returns:
        RecurringTaskPattern if found and owned by user, None otherwise.
    """
    statement = select(RecurringTaskPattern).where(
        RecurringTaskPattern.id == pattern_id,
        RecurringTaskPattern.user_id == user_id
    )
    result = await db.exec(statement)
    return result.first()


async def process_completed_recurring_task(
    db: AsyncSession,
    task_id: UUID,
    user_id: UUID
) -> Task | None:
    """Process a completed recurring task to generate the next occurrence.

    Args:
        db: Async database session.
        task_id: ID of the completed task.
        user_id: Owner's user ID.

    Returns:
        New task instance if created, None otherwise.
    """
    # Get the completed task
    completed_task = await get_task(db, task_id, user_id)
    if not completed_task or not completed_task.is_recurring or not completed_task.recurring_pattern_id:
        return None

    # Get the recurring pattern
    pattern = await get_recurring_task_pattern(db, completed_task.recurring_pattern_id, user_id)
    if not pattern:
        return None

    # Calculate next occurrence date based on pattern
    # For simplicity, we'll create the next occurrence the next day
    # In a real implementation, this would use more complex logic based on the pattern
    next_occurrence_date = datetime.now().date() + timedelta(days=1)

    # Create a new task based on the pattern
    new_task = Task(
        user_id=user_id,
        title=completed_task.title,
        description=completed_task.description,
        status=TaskStatus.PENDING,
        due_date=next_occurrence_date,
        priority=completed_task.priority,
        tags=completed_task.tags,
        is_recurring=True,
        recurring_pattern_id=pattern.id,
        parent_task_id=completed_task.id,
    )

    db.add(new_task)
    await db.commit()
    await db.refresh(new_task)

    # Update the completed task to link to the next occurrence
    completed_task.next_occurrence_id = new_task.id
    db.add(completed_task)
    await db.commit()

    return new_task
