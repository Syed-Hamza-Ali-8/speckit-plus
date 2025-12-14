"""Protected task endpoints with user ownership enforcement."""

from uuid import UUID

from fastapi import APIRouter, HTTPException, status

from app.api.deps import CurrentUser, DbSession
from app.schemas.task import TaskCreate, TaskResponse, TaskUpdate
from app.services import task_service

router = APIRouter(prefix="/tasks", tags=["tasks"])


@router.get("", response_model=list[TaskResponse])
async def list_tasks(
    db: DbSession,
    current_user: CurrentUser,
) -> list[TaskResponse]:
    """List all tasks for the current user.

    Returns only tasks owned by the authenticated user.
    """
    tasks = await task_service.get_tasks(db, current_user.id)
    return [TaskResponse.model_validate(task) for task in tasks]


@router.post("", response_model=TaskResponse, status_code=status.HTTP_201_CREATED)
async def create_task(
    data: TaskCreate,
    db: DbSession,
    current_user: CurrentUser,
) -> TaskResponse:
    """Create a new task for the current user.

    The task is automatically associated with the authenticated user.
    """
    task = await task_service.create_task(db, current_user.id, data)
    return TaskResponse.model_validate(task)


@router.get("/{task_id}", response_model=TaskResponse)
async def get_task(
    task_id: UUID,
    db: DbSession,
    current_user: CurrentUser,
) -> TaskResponse:
    """Get a specific task by ID.

    Returns 404 if task doesn't exist or is not owned by current user.
    """
    task = await task_service.get_task(db, task_id, current_user.id)
    if task is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found",
        )
    return TaskResponse.model_validate(task)


@router.patch("/{task_id}", response_model=TaskResponse)
async def update_task(
    task_id: UUID,
    data: TaskUpdate,
    db: DbSession,
    current_user: CurrentUser,
) -> TaskResponse:
    """Update a task by ID.

    Supports partial updates - only provided fields are updated.
    Returns 404 if task doesn't exist or is not owned by current user.
    """
    task = await task_service.update_task(db, task_id, current_user.id, data)
    if task is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found",
        )
    return TaskResponse.model_validate(task)


@router.delete("/{task_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_task(
    task_id: UUID,
    db: DbSession,
    current_user: CurrentUser,
) -> None:
    """Delete a task by ID.

    Returns 404 if task doesn't exist or is not owned by current user.
    """
    deleted = await task_service.delete_task(db, task_id, current_user.id)
    if not deleted:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found",
        )
