"""Protected task endpoints with user ownership enforcement and rate limiting."""

from datetime import date
from uuid import UUID

from fastapi import APIRouter, HTTPException, Query, Request, status

from app.api.deps import CurrentUser, DbSession
from app.middleware.rate_limit import (
    CREATE_TASK_RATE_LIMIT,
    DELETE_TASK_RATE_LIMIT,
    LIST_TASKS_RATE_LIMIT,
    UPDATE_TASK_RATE_LIMIT,
    limiter,
)
from app.models.task import TaskStatus
from app.schemas.task import (
    PaginatedTaskResponse,
    TaskCreate,
    TaskRead,
    TaskUpdate,
    SetTaskPriorityRequest,
    AddTaskTagsRequest,
    RemoveTaskTagsRequest,
    SearchTasksRequest,
    CreateRecurringTaskPatternRequest,
)
from app.services import task_service
from app.services import notification_service

router = APIRouter(prefix="/tasks", tags=["tasks"])

# Allowed sort fields for validation
ALLOWED_SORT_FIELDS = {"created_at", "updated_at", "title", "status"}


def parse_sort(sort: str) -> tuple[str, str]:
    """Parse and validate sort parameter string.

    Args:
        sort: Sort string in format "field:direction" (e.g., "created_at:desc")

    Returns:
        Tuple of (field, direction)

    Raises:
        HTTPException: If sort format is invalid or field/direction not allowed
    """
    parts = sort.split(":")
    if len(parts) != 2:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Sort must be in format 'field:direction'",
        )

    field, direction = parts
    if field not in ALLOWED_SORT_FIELDS:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid sort field: {field}. Allowed: {', '.join(sorted(ALLOWED_SORT_FIELDS))}",
        )
    if direction not in {"asc", "desc"}:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid sort direction: {direction}. Allowed: asc, desc",
        )

    return field, direction


@router.get("", response_model=PaginatedTaskResponse)
@limiter.limit(LIST_TASKS_RATE_LIMIT)
async def list_tasks(
    request: Request,
    db: DbSession,
    current_user: CurrentUser,
    status_filter: TaskStatus | None = Query(default=None, alias="status"),
    created_after: date | None = Query(default=None),
    created_before: date | None = Query(default=None),
    sort: str = Query(default="created_at:desc"),
    limit: int = Query(default=20, ge=1, le=100),
    offset: int = Query(default=0, ge=0),
) -> PaginatedTaskResponse:
    """List all tasks for the current user with filtering, sorting, and pagination.

    Returns only tasks owned by the authenticated user.
    Rate limited to 100 requests per hour per user.

    Query Parameters:
        status: Filter by task status (pending/completed)
        created_after: Filter tasks created after this date (ISO format)
        created_before: Filter tasks created before this date (ISO format)
        sort: Sort field and direction (format: field:direction, default: created_at:desc)
        limit: Number of items per page (1-100, default: 20)
        offset: Number of items to skip (default: 0)
    """
    # Parse and validate sort parameter
    sort_field, sort_direction = parse_sort(sort)

    # Get paginated tasks from service
    tasks, total = await task_service.get_tasks(
        db,
        current_user.id,
        status=status_filter,
        created_after=created_after,
        created_before=created_before,
        sort_field=sort_field,
        sort_direction=sort_direction,
        limit=limit,
        offset=offset,
    )

    return PaginatedTaskResponse(
        items=[TaskRead.model_validate(task) for task in tasks],
        total=total,
        limit=limit,
        offset=offset,
    )


@router.post("", response_model=TaskRead, status_code=status.HTTP_201_CREATED)
@limiter.limit(CREATE_TASK_RATE_LIMIT)
async def create_task(
    request: Request,
    data: TaskCreate,
    db: DbSession,
    current_user: CurrentUser,
) -> TaskRead:
    """Create a new task for the current user.

    The task is automatically associated with the authenticated user.
    Rate limited to 30 requests per hour per user.
    """
    task = await task_service.create_task(db, current_user.id, data)

    # Create notification for task creation (only if has due date)
    if task.due_date:
        await notification_service.notify_task_created(
            db=db,
            user_id=current_user.id,
            task_title=task.title,
            task_id=task.id,
            due_date=task.due_date,
        )

    return TaskRead.model_validate(task)


@router.get("/{task_id}", response_model=TaskRead)
async def get_task(
    task_id: UUID,
    db: DbSession,
    current_user: CurrentUser,
) -> TaskRead:
    """Get a specific task by ID.

    Returns 404 if task doesn't exist or is not owned by current user.
    """
    task = await task_service.get_task(db, task_id, current_user.id)
    if task is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found",
        )
    return TaskRead.model_validate(task)


@router.patch("/{task_id}", response_model=TaskRead)
@limiter.limit(UPDATE_TASK_RATE_LIMIT)
async def update_task(
    request: Request,
    task_id: UUID,
    data: TaskUpdate,
    db: DbSession,
    current_user: CurrentUser,
) -> TaskRead:
    """Update a task by ID.

    Supports partial updates - only provided fields are updated.
    Returns 404 if task doesn't exist or is not owned by current user.
    Rate limited to 60 requests per hour per user.
    """
    # Get current task state before update for comparison
    old_task = await task_service.get_task(db, task_id, current_user.id)
    if old_task is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found",
        )
    old_status = old_task.status

    task = await task_service.update_task(db, task_id, current_user.id, data)
    if task is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found",
        )

    # Create notification if task was just completed
    if data.status == TaskStatus.COMPLETED and old_status != TaskStatus.COMPLETED:
        await notification_service.notify_task_completed(
            db=db,
            user_id=current_user.id,
            task_title=task.title,
            task_id=task.id,
        )

    return TaskRead.model_validate(task)


@router.delete("/{task_id}", status_code=status.HTTP_204_NO_CONTENT)
@limiter.limit(DELETE_TASK_RATE_LIMIT)
async def delete_task(
    request: Request,
    task_id: UUID,
    db: DbSession,
    current_user: CurrentUser,
) -> None:
    """Delete a task by ID.

    Returns 404 if task doesn't exist or is not owned by current user.
    Rate limited to 30 requests per hour per user.
    """
    # Get task info before deletion for notification
    task = await task_service.get_task(db, task_id, current_user.id)
    if task is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found",
        )
    task_title = task.title

    deleted = await task_service.delete_task(db, task_id, current_user.id)
    if not deleted:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found",
        )

    # Create notification for task deletion
    await notification_service.notify_task_deleted(
        db=db,
        user_id=current_user.id,
        task_title=task_title,
    )


@router.patch("/{task_id}/priority", response_model=TaskRead)
@limiter.limit(UPDATE_TASK_RATE_LIMIT)
async def set_task_priority(
    request: Request,
    task_id: UUID,
    data: SetTaskPriorityRequest,
    db: DbSession,
    current_user: CurrentUser,
) -> TaskRead:
    """Set priority for a specific task.

    Updates the priority level of a task.
    Returns 404 if task doesn't exist or is not owned by current user.
    Rate limited to 60 requests per hour per user.
    """
    task = await task_service.get_task(db, task_id, current_user.id)
    if task is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found",
        )

    updated_task = await task_service.set_task_priority(
        db,
        task_id,
        current_user.id,
        data.priority
    )

    if updated_task is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found",
        )

    return TaskRead.model_validate(updated_task)


@router.patch("/{task_id}/tags", response_model=TaskRead)
@limiter.limit(UPDATE_TASK_RATE_LIMIT)
async def add_task_tags(
    request: Request,
    task_id: UUID,
    data: AddTaskTagsRequest,
    db: DbSession,
    current_user: CurrentUser,
) -> TaskRead:
    """Add tags to a specific task.

    Adds new tags to a task's existing tags.
    Returns 404 if task doesn't exist or is not owned by current user.
    Rate limited to 60 requests per hour per user.
    """
    task = await task_service.get_task(db, task_id, current_user.id)
    if task is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found",
        )

    updated_task = await task_service.add_task_tags(
        db,
        task_id,
        current_user.id,
        data.tags
    )

    if updated_task is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found",
        )

    return TaskRead.model_validate(updated_task)


@router.delete("/{task_id}/tags", response_model=TaskRead)
@limiter.limit(UPDATE_TASK_RATE_LIMIT)
async def remove_task_tags(
    request: Request,
    task_id: UUID,
    data: RemoveTaskTagsRequest,
    db: DbSession,
    current_user: CurrentUser,
) -> TaskRead:
    """Remove tags from a specific task.

    Removes specified tags from a task's existing tags.
    Returns 404 if task doesn't exist or is not owned by current user.
    Rate limited to 60 requests per hour per user.
    """
    task = await task_service.get_task(db, task_id, current_user.id)
    if task is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found",
        )

    updated_task = await task_service.remove_task_tags(
        db,
        task_id,
        current_user.id,
        data.tags
    )

    if updated_task is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found",
        )

    return TaskRead.model_validate(updated_task)


@router.get("/search", response_model=PaginatedTaskResponse)
@limiter.limit(LIST_TASKS_RATE_LIMIT)
async def search_tasks(
    request: Request,
    db: DbSession,
    current_user: CurrentUser,
    query: str = Query(..., min_length=1, description="Search query string"),
    status_filter: TaskStatus | None = Query(default=None, alias="status"),
    priority: str | None = Query(default=None, description="Filter by priority level"),
    tags: list[str] | None = Query(default=None, description="Filter by tags"),
    due_before: date | None = Query(default=None, description="Filter by due date before"),
    due_after: date | None = Query(default=None, description="Filter by due date after"),
    sort_by: str = Query(default="created_at", description="Field to sort by"),
    order: str = Query(default="desc", description="Sort order (asc or desc)"),
    page: int = Query(default=1, ge=1, description="Page number"),
    per_page: int = Query(default=20, ge=1, le=100, description="Number of items per page"),
) -> PaginatedTaskResponse:
    """Search tasks by query string and filters.

    Returns tasks matching the search query and filters.
    Rate limited to 100 requests per hour per user.
    """
    # Calculate offset based on page and per_page
    offset = (page - 1) * per_page

    # Get paginated tasks from service with search and filters
    tasks, total = await task_service.search_tasks(
        db,
        current_user.id,
        query=query,
        status=status_filter,
        priority=priority,
        tags=tags,
        due_before=due_before,
        due_after=due_after,
        sort_by=sort_by,
        order=order,
        page=page,
        per_page=per_page,
    )

    return PaginatedTaskResponse(
        items=[TaskRead.model_validate(task) for task in tasks],
        total=total,
        limit=per_page,
        offset=offset,
    )


@router.post("/recurring-patterns", response_model=dict)
@limiter.limit(CREATE_TASK_RATE_LIMIT)
async def create_recurring_task_pattern(
    request: Request,
    data: CreateRecurringTaskPatternRequest,
    db: DbSession,
    current_user: CurrentUser,
) -> dict:
    """Create a recurring task pattern.

    Creates a recurring task pattern that can generate tasks based on the specified pattern.
    Rate limited to 30 requests per hour per user.
    """
    pattern = await task_service.create_recurring_task_pattern(
        db,
        current_user.id,
        data.base_task_title,
        data.base_task_description,
        data.pattern_type,
        data.interval,
        data.start_date,
        data.end_date,
        data.weekdays,
        data.days_of_month
    )

    return {
        "id": pattern.id,
        "base_task_title": pattern.base_task_title,
        "pattern_type": pattern.pattern_type,
        "interval": pattern.interval,
        "start_date": pattern.start_date,
        "end_date": pattern.end_date,
        "weekdays": pattern.weekdays,
        "days_of_month": pattern.days_of_month,
        "created_at": pattern.created_at,
    }
