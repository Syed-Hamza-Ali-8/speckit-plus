from fastapi import APIRouter, Depends, HTTPException, status, Query
from sqlmodel import Session, select, func, or_, and_
from typing import List, Optional
from datetime import datetime, date
from uuid import UUID
import json

from ..models.task_models import (
    Task,
    TaskUpdate,
    TaskRead,
    TaskStatus,
    PriorityLevel,
    Tag,
    TagCreate,
    TagRead,
    TaskSearchRequest,
    TaskSearchResponse
)
from ..config.database import get_session


router = APIRouter(prefix="", tags=["advanced-tasks"])


@router.put("/tasks/{task_id}/priority")
def set_task_priority(
    task_id: UUID,
    priority: PriorityLevel,
    session: Session = Depends(get_session)
):
    """
    Set the priority level for a task.
    """
    try:
        task = session.get(Task, task_id)
        if not task:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Task not found"
            )

        task.priority = priority
        session.add(task)
        session.commit()
        session.refresh(task)

        return {
            "task_id": task.id,
            "priority": task.priority,
            "message": "Priority updated successfully"
        }

    except HTTPException:
        raise
    except Exception as e:
        session.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating task priority: {str(e)}"
        )


@router.put("/tasks/{task_id}/tags")
def add_task_tags(
    task_id: UUID,
    tags: List[str],
    session: Session = Depends(get_session)
):
    """
    Add tags to a task.
    """
    try:
        task = session.get(Task, task_id)
        if not task:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Task not found"
            )

        # Get existing tags and add new ones
        existing_tags = set(task.tags)
        new_tags = set(tags)
        all_tags = list(existing_tags.union(new_tags))

        task.tags = all_tags
        session.add(task)
        session.commit()
        session.refresh(task)

        return {
            "task_id": task.id,
            "tags": task.tags,
            "message": "Tags added successfully"
        }

    except HTTPException:
        raise
    except Exception as e:
        session.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error adding tags: {str(e)}"
        )


@router.delete("/tasks/{task_id}/tags")
def remove_task_tags(
    task_id: UUID,
    tags: List[str],
    session: Session = Depends(get_session)
):
    """
    Remove tags from a task.
    """
    try:
        task = session.get(Task, task_id)
        if not task:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Task not found"
            )

        # Remove specified tags
        current_tags = set(task.tags)
        tags_to_remove = set(tags)
        remaining_tags = list(current_tags - tags_to_remove)

        task.tags = remaining_tags
        session.add(task)
        session.commit()
        session.refresh(task)

        return {
            "task_id": task.id,
            "tags": task.tags,
            "message": "Tags removed successfully"
        }

    except HTTPException:
        raise
    except Exception as e:
        session.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error removing tags: {str(e)}"
        )


@router.get("/users/{user_id}/tags", response_model=List[TagRead])
def get_user_tags(
    user_id: UUID,
    session: Session = Depends(get_session)
):
    """
    Get all tags for a user.
    """
    try:
        statement = select(Tag).where(Tag.user_id == user_id)
        user_tags = session.exec(statement).all()
        return user_tags

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving user tags: {str(e)}"
        )


@router.post("/users/{user_id}/tags", response_model=TagRead)
def create_user_tag(
    user_id: UUID,
    tag: TagCreate,
    session: Session = Depends(get_session)
):
    """
    Create a new tag for a user.
    """
    try:
        # Check if tag already exists for this user
        statement = select(Tag).where(
            Tag.user_id == user_id,
            Tag.name == tag.name
        )
        existing_tag = session.exec(statement).first()

        if existing_tag:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Tag with this name already exists for this user"
            )

        db_tag = Tag.model_validate(tag)
        session.add(db_tag)
        session.commit()
        session.refresh(db_tag)

        return db_tag

    except HTTPException:
        raise
    except Exception as e:
        session.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating tag: {str(e)}"
        )


@router.get("/users/{user_id}/tasks/search", response_model=TaskSearchResponse)
def search_tasks(
    user_id: UUID,
    query: str = Query(..., min_length=1),
    status_filter: Optional[str] = Query(None, regex="^(all|pending|completed)$"),
    priority: Optional[PriorityLevel] = None,
    tags: List[str] = Query([]),
    due_before: Optional[date] = None,
    due_after: Optional[date] = None,
    sort_by: str = Query("created_at", regex="^(created_at|title|due_date|priority)$"),
    order: str = Query("desc", regex="^(asc|desc)$"),
    page: int = Query(1, ge=1),
    per_page: int = Query(20, ge=1, le=100),
    session: Session = Depends(get_session)
):
    """
    Search tasks with various filters and sorting options.
    """
    try:
        # Build the query with filters
        statement = select(Task).where(Task.user_id == user_id)

        # Apply status filter
        if status_filter and status_filter != "all":
            if status_filter == "pending":
                statement = statement.where(Task.status == TaskStatus.PENDING)
            elif status_filter == "completed":
                statement = statement.where(Task.status == TaskStatus.COMPLETED)

        # Apply priority filter
        if priority:
            statement = statement.where(Task.priority == priority)

        # Apply tags filter
        if tags:
            # Filter tasks that contain all specified tags
            for tag in tags:
                statement = statement.where(func.array_to_string(Task.tags, ',').contains(tag))

        # Apply due date filters
        if due_before:
            statement = statement.where(Task.due_date <= due_before)
        if due_after:
            statement = statement.where(Task.due_date >= due_after)

        # Apply text search
        if query:
            # Search in title and description
            statement = statement.where(
                or_(
                    Task.title.ilike(f"%{query}%"),
                    Task.description.ilike(f"%{query}%")
                )
            )

        # Apply sorting
        if sort_by == "created_at":
            if order == "asc":
                statement = statement.order_by(Task.created_at.asc())
            else:
                statement = statement.order_by(Task.created_at.desc())
        elif sort_by == "title":
            if order == "asc":
                statement = statement.order_by(Task.title.asc())
            else:
                statement = statement.order_by(Task.title.desc())
        elif sort_by == "due_date":
            if order == "asc":
                statement = statement.order_by(Task.due_date.asc())
            else:
                statement = statement.order_by(Task.due_date.desc())
        elif sort_by == "priority":
            if order == "asc":
                statement = statement.order_by(Task.priority.asc())
            else:
                statement = statement.order_by(Task.priority.desc())

        # Get total count
        count_statement = select(func.count()).select_from(statement.subquery())
        total_count = session.exec(count_statement).one()

        # Apply pagination
        offset = (page - 1) * per_page
        statement = statement.offset(offset).limit(per_page)

        # Execute query
        tasks = session.exec(statement).all()

        return TaskSearchResponse(
            results=[TaskRead.from_orm(task) if hasattr(TaskRead, 'from_orm') else TaskRead.model_validate(task) for task in tasks],
            total_count=total_count,
            page=page,
            per_page=per_page
        )

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error searching tasks: {str(e)}"
        )


@router.get("/users/{user_id}/tasks/filter", response_model=List[TaskRead])
def filter_tasks(
    user_id: UUID,
    status: Optional[str] = Query(None, regex="^(all|pending|completed)$"),
    priority: Optional[PriorityLevel] = None,
    tags: List[str] = Query([]),
    due_before: Optional[date] = None,
    due_after: Optional[date] = None,
    session: Session = Depends(get_session)
):
    """
    Filter tasks by various criteria.
    """
    try:
        statement = select(Task).where(Task.user_id == user_id)

        # Apply filters
        if status and status != "all":
            if status == "pending":
                statement = statement.where(Task.status == TaskStatus.PENDING)
            elif status == "completed":
                statement = statement.where(Task.status == TaskStatus.COMPLETED)

        if priority:
            statement = statement.where(Task.priority == priority)

        if tags:
            # Filter tasks that contain all specified tags
            for tag in tags:
                statement = statement.where(func.array_to_string(Task.tags, ',').contains(tag))

        if due_before:
            statement = statement.where(Task.due_date <= due_before)
        if due_after:
            statement = statement.where(Task.due_date >= due_after)

        # Execute query
        tasks = session.exec(statement).all()
        return [TaskRead.from_orm(task) if hasattr(TaskRead, 'from_orm') else TaskRead.model_validate(task) for task in tasks]

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error filtering tasks: {str(e)}"
        )


@router.get("/users/{user_id}/tasks/sort", response_model=List[TaskRead])
def sort_tasks(
    user_id: UUID,
    sort_by: str = Query("created_at", regex="^(created_at|title|due_date|priority)$"),
    order: str = Query("desc", regex="^(asc|desc)$"),
    session: Session = Depends(get_session)
):
    """
    Sort tasks by various criteria.
    """
    try:
        statement = select(Task).where(Task.user_id == user_id)

        # Apply sorting
        if sort_by == "created_at":
            if order == "asc":
                statement = statement.order_by(Task.created_at.asc())
            else:
                statement = statement.order_by(Task.created_at.desc())
        elif sort_by == "title":
            if order == "asc":
                statement = statement.order_by(Task.title.asc())
            else:
                statement = statement.order_by(Task.title.desc())
        elif sort_by == "due_date":
            if order == "asc":
                statement = statement.order_by(Task.due_date.asc())
            else:
                statement = statement.order_by(Task.due_date.desc())
        elif sort_by == "priority":
            if order == "asc":
                statement = statement.order_by(Task.priority.asc())
            else:
                statement = statement.order_by(Task.priority.desc())

        # Execute query
        tasks = session.exec(statement).all()
        return [TaskRead.from_orm(task) if hasattr(TaskRead, 'from_orm') else TaskRead.model_validate(task) for task in tasks]

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error sorting tasks: {str(e)}"
        )