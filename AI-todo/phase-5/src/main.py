from fastapi import FastAPI, Depends, HTTPException, status, Header
from sqlmodel import Session, SQLModel, create_engine, select
from sqlalchemy import func, and_
from contextlib import asynccontextmanager
from typing import AsyncGenerator
import asyncio
import logging
from typing import Optional, List
from datetime import date, datetime
from uuid import uuid4, UUID
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware

from config.database import get_session, engine
from api.auth import router as auth_router
from api.notifications import router as notifications_router
from api.chat import router as chat_router
from api.recurring_tasks import router as recurring_tasks_router
from api.reminders import router as reminders_router
from api.tasks_advanced import router as tasks_advanced_router
from models.user_models import User
from models.task_models import Task, RecurringTaskPattern, Reminder, Tag, TaskStatus, TaskUpdate
from models.notification_models import Notification
from services.kafka_producer import init_kafka_producer, close_kafka_producer
from services.dapr_service import dapr_service
from services.notification_helper import (
    create_task_created_notification,
    create_task_updated_notification,
    create_task_deleted_notification,
    create_task_completed_notification
)
from passlib.context import CryptContext


# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    """
    Application lifespan manager for startup and shutdown events.
    """
    logger.info("Starting up Phase V Todo App with advanced features...")

    # Initialize Kafka producer (optional - gracefully handle if not available)
    try:
        await init_kafka_producer()
        logger.info("Kafka producer initialized")
    except Exception as e:
        logger.warning(f"Kafka producer not available: {e}")

    # Create database tables
    SQLModel.metadata.create_all(bind=engine)
    logger.info("Database tables created")

    # Initialize Dapr components (optional)
    logger.info("Dapr service initialized")

    yield

    # Shutdown
    logger.info("Shutting down Phase V Todo App...")
    try:
        await close_kafka_producer()
        logger.info("Kafka producer closed")
    except Exception:
        pass


# Request/Response schemas
class TaskCreateRequest(BaseModel):
    title: str
    description: Optional[str] = None
    due_date: Optional[date] = None
    priority: str = "medium"
    tags: List[str] = []
    is_recurring: bool = False


# Create FastAPI app with lifespan - using same structure as Phase 2
app = FastAPI(
    title="Phase V Todo App - Advanced Features",
    description="Todo app with advanced features: recurring tasks, due dates, reminders, priorities, tags, search, filter, and sort",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware to allow frontend requests
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Include API routers - add advanced features to existing structure
# Add authentication endpoints
app.include_router(auth_router)
# Add notifications endpoints
app.include_router(notifications_router)
# Add chat endpoints
app.include_router(chat_router)
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
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """
    Get all tasks with filtering, sorting, and pagination (Phase 2 compatible).
    Extended with advanced filtering options.
    Requires JWT authentication - users can only see their own tasks.
    """
    from core.security import decode_access_token

    # Extract and validate JWT token
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required"
        )

    token = authorization.split(" ")[1]
    payload = decode_access_token(token)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token"
        )

    user_id_str = payload.get("sub")
    if not user_id_str:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token payload"
        )

    try:
        authenticated_user_id = UUID(user_id_str)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid user ID in token"
        )

    # Filter tasks by authenticated user
    statement = select(Task).where(Task.user_id == authenticated_user_id)

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

    # Get total count using func.count instead of .count()
    count_base = select(Task)
    count_conditions = []
    if status_filter:
        count_conditions.append(Task.status == status_filter)
    if created_after:
        count_conditions.append(Task.created_at >= created_after)
    if created_before:
        count_conditions.append(Task.created_at <= created_before)
    if count_conditions:
        count_base = count_base.where(and_(*count_conditions))
    count_statement = select(func.count()).select_from(count_base.subquery())
    total = session.exec(count_statement).one() or 0

    return {
        "items": tasks,
        "total": total,
        "limit": limit,
        "offset": offset
    }


@app.post("/tasks")
async def create_task(
    request: TaskCreateRequest,
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """
    Create a new task (Phase 2 compatible).
    Extended with advanced features.
    Requires JWT authentication.
    """
    from models.user_models import User
    from core.security import decode_access_token

    # Extract and validate JWT token - REQUIRED (no fallback for security)
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required"
        )

    token = authorization.split(" ")[1]
    payload = decode_access_token(token)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token"
        )

    user_id_str = payload.get("sub")
    if not user_id_str:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token payload"
        )

    try:
        user_id = UUID(user_id_str)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid user ID in token"
        )

    # Get user from database
    user = session.get(User, user_id)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )

    task = Task(
        title=request.title,
        description=request.description,
        due_date=request.due_date,
        priority=request.priority,
        tags=request.tags,
        is_recurring=request.is_recurring,
        status=TaskStatus.PENDING,
        user_id=user.id,
        created_by=user.id,
        updated_by=user.id,
    )

    session.add(task)
    session.commit()
    session.refresh(task)

    # Create notification for task creation
    try:
        create_task_created_notification(
            session=session,
            user_id=task.user_id,
            task_id=task.id,
            task_title=task.title
        )
    except Exception as e:
        logger.error(f"Failed to create notification: {e}")

    # Publish event if Kafka is available
    try:
        from services.kafka_producer import publish_task_created_event
        await publish_task_created_event(task.model_dump(), str(task.user_id))
    except Exception as e:
        logger.error(f"Failed to publish task created event: {e}")

    # Publish event via Dapr if available
    try:
        from services.dapr_service import publish_task_event_dapr
        await publish_task_event_dapr("created", task.model_dump(), str(task.user_id))
    except Exception as e:
        logger.error(f"Failed to publish task created event via Dapr: {e}")

    return task


@app.get("/tasks/{task_id}")
async def get_task_by_id(
    task_id: UUID,
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """
    Get a specific task by ID (Phase 2 compatible).
    Requires JWT authentication - users can only view their own tasks.
    """
    from core.security import decode_access_token

    # Extract and validate JWT token
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required"
        )

    token = authorization.split(" ")[1]
    payload = decode_access_token(token)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token"
        )

    user_id_str = payload.get("sub")
    if not user_id_str:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token payload"
        )

    try:
        authenticated_user_id = UUID(user_id_str)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid user ID in token"
        )

    task = session.get(Task, task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found"
        )

    # Verify the task belongs to the authenticated user
    if task.user_id != authenticated_user_id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied"
        )

    return task


@app.patch("/tasks/{task_id}")
async def update_task(
    task_id: UUID,
    task_update: TaskUpdate,
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """
    Update a task by ID (Phase 2 compatible).
    Extended with advanced features.
    Requires JWT authentication - users can only update their own tasks.
    """
    from core.security import decode_access_token

    # Extract and validate JWT token
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required"
        )

    token = authorization.split(" ")[1]
    payload = decode_access_token(token)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token"
        )

    user_id_str = payload.get("sub")
    if not user_id_str:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token payload"
        )

    try:
        authenticated_user_id = UUID(user_id_str)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid user ID in token"
        )

    task = session.get(Task, task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found"
        )

    # Verify the task belongs to the authenticated user
    if task.user_id != authenticated_user_id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied"
        )

    # Update fields if provided (only update non-None values)
    update_data = task_update.dict(exclude_unset=True)

    # Check if status is being changed to completed
    status_changed_to_completed = (
        'status' in update_data and
        update_data['status'] == TaskStatus.COMPLETED and
        task.status != TaskStatus.COMPLETED
    )

    for field, value in update_data.items():
        setattr(task, field, value)

    # Update the updated_at timestamp
    task.updated_at = datetime.utcnow()

    session.add(task)
    session.commit()
    session.refresh(task)

    # Create notification for task update
    try:
        if status_changed_to_completed:
            # Special notification for task completion
            create_task_completed_notification(
                session=session,
                user_id=task.user_id,
                task_id=task.id,
                task_title=task.title
            )
        else:
            # General update notification
            create_task_updated_notification(
                session=session,
                user_id=task.user_id,
                task_id=task.id,
                task_title=task.title
            )
    except Exception as e:
        logger.error(f"Failed to create notification: {e}")

    # Publish event if Kafka is available
    try:
        from services.kafka_producer import publish_task_updated_event
        await publish_task_updated_event(task.dict(), str(task.user_id))
    except Exception as e:
        logger.error(f"Failed to publish task updated event: {e}")

    return task


@app.delete("/tasks/{task_id}")
async def delete_task(
    task_id: UUID,
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """
    Delete a task by ID (Phase 2 compatible).
    Requires JWT authentication - users can only delete their own tasks.
    """
    from core.security import decode_access_token

    # Extract and validate JWT token
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required"
        )

    token = authorization.split(" ")[1]
    payload = decode_access_token(token)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token"
        )

    user_id_str = payload.get("sub")
    if not user_id_str:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token payload"
        )

    try:
        authenticated_user_id = UUID(user_id_str)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid user ID in token"
        )

    task = session.get(Task, task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found"
        )

    # Verify the task belongs to the authenticated user
    if task.user_id != authenticated_user_id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied"
        )

    # Store task info before deletion
    task_title = task.title
    task_user_id = task.user_id

    # Delete all notifications related to this task first (to avoid foreign key constraint)
    try:
        from models.notification_models import Notification
        notification_statement = select(Notification).where(Notification.task_id == task_id)
        notifications = session.exec(notification_statement).all()
        for notification in notifications:
            session.delete(notification)
        session.commit()
        logger.info(f"Deleted {len(notifications)} notifications for task {task_id}")
    except Exception as e:
        logger.error(f"Failed to delete notifications: {e}")
        session.rollback()

    # Create a final deletion notification (without task_id reference to avoid constraint)
    try:
        create_task_deleted_notification(
            session=session,
            user_id=task_user_id,
            task_id=None,  # Don't reference the task being deleted
            task_title=task_title
        )
    except Exception as e:
        logger.error(f"Failed to create notification: {e}")

    # Now delete the task
    session.delete(task)
    session.commit()

    # Publish event if Kafka is available
    try:
        from .services.kafka_producer import publish_task_deleted_event
        await publish_task_deleted_event(task_id, str(task_user_id))
    except Exception as e:
        logger.error(f"Failed to publish task deleted event: {e}")

    return {"message": "Task deleted successfully"}


# Test user endpoint for local development
@app.post("/test-user")
async def create_test_user(
    email: str = "test@example.com",
    name: str = "Test User",
    session: Session = Depends(get_session)
):
    """Create a test user for local development."""
    from models.user_models import User

    # Use bcrypt directly
    import bcrypt
    raw_password = "pass"
    password_hash = bcrypt.hashpw(raw_password.encode('utf-8'), bcrypt.gensalt()).decode('utf-8')

    user = User(
        email=email,
        name=name,
        password_hash=password_hash,
        is_active=True
    )

    session.add(user)
    session.commit()
    session.refresh(user)

    return {
        "message": "Test user created",
        "user_id": str(user.id),
        "email": user.email,
        "name": user.name
    }


@app.get("/test-users")
async def get_test_users(session: Session = Depends(get_session)):
    """Get all test users."""
    from models.user_models import User

    statement = select(User)
    users = session.exec(statement).all()

    return {"users": users}


# Error handling
@app.exception_handler(Exception)
def global_exception_handler(request, exc):
    """
    Global exception handler.
    """
    logger.error(f"Unhandled exception: {exc}")
    from fastapi.responses import JSONResponse
    return JSONResponse(
        status_code=500,
        content={"message": "An error occurred", "details": str(exc)}
    )


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)