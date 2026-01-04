from fastapi import FastAPI, Depends, HTTPException, status
from sqlmodel import Session, SQLModel, create_engine, select
from contextlib import asynccontextmanager
from typing import AsyncGenerator
import asyncio
import logging
from datetime import date

from .config.database import get_session, engine
from .api.recurring_tasks import router as recurring_tasks_router
from .api.reminders import router as reminders_router
from .api.tasks_advanced import router as tasks_advanced_router
from .models.task_models import Task, RecurringTaskPattern, Reminder, Tag, TaskStatus
from .services.kafka_producer import init_kafka_producer, close_kafka_producer
from .services.dapr_service import dapr_service


# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    """
    Application lifespan manager for startup and shutdown events.
    """
    logger.info("Starting up Phase V Todo App with advanced features...")

    # Initialize Kafka producer
    await init_kafka_producer()
    logger.info("Kafka producer initialized")

    # Create database tables
    SQLModel.metadata.create_all(bind=engine)
    logger.info("Database tables created")

    # Initialize Dapr components
    logger.info("Dapr service initialized")

    yield

    # Shutdown
    logger.info("Shutting down Phase V Todo App...")
    await close_kafka_producer()
    logger.info("Kafka producer closed")


# Create FastAPI app with lifespan - using same structure as Phase 2
app = FastAPI(
    title="Phase V Todo App - Advanced Features",
    description="Todo app with advanced features: recurring tasks, due dates, reminders, priorities, tags, search, filter, and sort",
    version="1.0.0",
    lifespan=lifespan
)


# Include API routers - add advanced features to existing structure
# Add recurring tasks endpoints
app.include_router(recurring_tasks_router)
# Add reminders endpoints
app.include_router(reminders_router)
# Add advanced task features
app.include_router(tasks_advanced_router)


@app.get("/")
async def root():
    """
    Root endpoint for the Phase V Todo App.
    """
    return {
        "message": "Welcome to Phase V Todo App",
        "features": [
            "Recurring Tasks",
            "Due Dates & Reminders",
            "Priorities & Tags",
            "Search & Filter",
            "Sort Tasks",
            "Event-Driven Architecture with Kafka",
            "Dapr Integration"
        ]
    }


@app.get("/health")
async def health_check():
    """
    Health check endpoint.
    """
    return {
        "status": "healthy",
        "phase": "V",
        "features": "advanced",
        "timestamp": "datetime.utcnow().isoformat()"
    }


# Phase 2 compatible task endpoints with advanced features
@app.get("/tasks")
async def get_tasks(
    status_filter: TaskStatus = None,
    created_after: date = None,
    created_before: date = None,
    sort: str = "created_at:desc",
    limit: int = 20,
    offset: int = 0,
    session: Session = Depends(get_session)
):
    """
    Get all tasks with filtering, sorting, and pagination (Phase 2 compatible).
    Extended with advanced filtering options.
    """
    statement = select(Task)

    if status_filter:
        statement = statement.where(Task.status == status_filter)
    if created_after:
        statement = statement.where(Task.created_at >= created_after)
    if created_before:
        statement = statement.where(Task.created_at <= created_before)

    # Additional advanced filters
    # These can be added as query parameters if needed

    # Apply sorting
    sort_parts = sort.split(":")
    if len(sort_parts) == 2:
        field, direction = sort_parts
        if field in ["created_at", "updated_at", "title"]:
            if direction == "desc":
                statement = statement.order_by(getattr(Task, field).desc())
            else:
                statement = statement.order_by(getattr(Task, field).asc())

    # Apply pagination
    statement = statement.offset(offset).limit(limit)

    tasks = session.exec(statement).all()

    # Get total count
    count_statement = select(Task)
    if status_filter:
        count_statement = count_statement.where(Task.status == status_filter)
    if created_after:
        count_statement = count_statement.where(Task.created_at >= created_after)
    if created_before:
        count_statement = count_statement.where(Task.created_at <= created_before)

    total = session.exec(count_statement).count()

    return {
        "items": tasks,
        "total": total,
        "limit": limit,
        "offset": offset
    }


@app.post("/tasks")
async def create_task(
    title: str,
    description: str = None,
    due_date: date = None,
    priority: str = "medium",  # Advanced feature
    tags: list = [],  # Advanced feature
    is_recurring: bool = False,  # Advanced feature
    recurring_pattern_id: int = None,  # Advanced feature
    session: Session = Depends(get_session)
):
    """
    Create a new task (Phase 2 compatible).
    Extended with advanced features.
    """
    task = Task(
        title=title,
        description=description,
        due_date=due_date,
        priority=priority,
        tags=tags,
        is_recurring=is_recurring,
        recurring_pattern_id=recurring_pattern_id,
        status=TaskStatus.PENDING
    )

    session.add(task)
    session.commit()
    session.refresh(task)

    # Publish event if Kafka is available
    try:
        from .services.kafka_producer import publish_task_created_event
        await publish_task_created_event(task.model_dump(), str(task.user_id))
    except Exception as e:
        logger.error(f"Failed to publish task created event: {e}")

    # Publish event via Dapr if available
    try:
        from .services.dapr_service import publish_task_event_dapr
        await publish_task_event_dapr("created", task.model_dump(), str(task.user_id))
    except Exception as e:
        logger.error(f"Failed to publish task created event via Dapr: {e}")

    return task


@app.get("/tasks/{task_id}")
async def get_task_by_id(
    task_id: int,
    session: Session = Depends(get_session)
):
    """
    Get a specific task by ID (Phase 2 compatible).
    """
    task = session.get(Task, task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found"
        )

    return task


@app.patch("/tasks/{task_id}")
async def update_task(
    task_id: int,
    title: str = None,
    description: str = None,
    status: TaskStatus = None,
    due_date: date = None,
    priority: str = None,  # Advanced feature
    tags: list = None,  # Advanced feature
    session: Session = Depends(get_session)
):
    """
    Update a task by ID (Phase 2 compatible).
    Extended with advanced features.
    """
    task = session.get(Task, task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found"
        )

    # Update fields if provided
    if title is not None:
        task.title = title
    if description is not None:
        task.description = description
    if status is not None:
        task.status = status
    if due_date is not None:
        task.due_date = due_date
    if priority is not None:
        task.priority = priority
    if tags is not None:
        task.tags = tags

    session.add(task)
    session.commit()
    session.refresh(task)

    # Publish event if Kafka is available
    try:
        from .services.kafka_producer import publish_task_updated_event
        await publish_task_updated_event(task.model_dump(), str(task.user_id))
    except Exception as e:
        logger.error(f"Failed to publish task updated event: {e}")

    return task


@app.delete("/tasks/{task_id}")
async def delete_task(
    task_id: int,
    session: Session = Depends(get_session)
):
    """
    Delete a task by ID (Phase 2 compatible).
    """
    task = session.get(Task, task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found"
        )

    session.delete(task)
    session.commit()

    # Publish event if Kafka is available
    try:
        from .services.kafka_producer import publish_task_deleted_event
        await publish_task_deleted_event(task_id, str(task.user_id))
    except Exception as e:
        logger.error(f"Failed to publish task deleted event: {e}")

    return {"message": "Task deleted successfully"}


# Error handling
@app.exception_handler(Exception)
async def global_exception_handler(request, exc):
    """
    Global exception handler.
    """
    logger.error(f"Unhandled exception: {exc}")
    return {"message": "An error occurred", "details": str(exc)}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)