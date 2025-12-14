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
)
from app.services import task_service

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
    task = await task_service.update_task(db, task_id, current_user.id, data)
    if task is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found",
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
    deleted = await task_service.delete_task(db, task_id, current_user.id)
    if not deleted:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found",
        )
