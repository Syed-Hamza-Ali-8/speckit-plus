"""Helper functions for creating notifications."""
import logging
from uuid import UUID
from sqlmodel import Session
from models.notification_models import Notification, NotificationType

logger = logging.getLogger(__name__)


def create_notification(
    session: Session,
    user_id: UUID,
    title: str,
    message: str,
    notification_type: NotificationType,
    task_id: UUID | None = None
) -> Notification | None:
    """
    Create a notification for a user.

    Args:
        session: Database session
        user_id: User ID to notify
        title: Notification title
        message: Notification message
        notification_type: Type of notification
        task_id: Optional task ID related to the notification

    Returns:
        Created notification or None if failed
    """
    try:
        notification = Notification(
            user_id=user_id,
            title=title,
            message=message,
            type=notification_type,
            task_id=task_id
        )
        session.add(notification)
        session.commit()
        session.refresh(notification)
        logger.info(f"Created notification {notification.id} for user {user_id}")
        return notification
    except Exception as e:
        logger.error(f"Failed to create notification: {e}")
        session.rollback()
        return None


def create_task_created_notification(
    session: Session,
    user_id: UUID,
    task_id: UUID,
    task_title: str
) -> Notification | None:
    """Create a notification when a task is created."""
    return create_notification(
        session=session,
        user_id=user_id,
        title="Task Created",
        message=f'Task "{task_title}" has been created',
        notification_type=NotificationType.TASK_CREATED,
        task_id=task_id
    )


def create_task_updated_notification(
    session: Session,
    user_id: UUID,
    task_id: UUID,
    task_title: str
) -> Notification | None:
    """Create a notification when a task is updated."""
    return create_notification(
        session=session,
        user_id=user_id,
        title="Task Updated",
        message=f'Task "{task_title}" has been updated',
        notification_type=NotificationType.TASK_UPDATED,
        task_id=task_id
    )


def create_task_deleted_notification(
    session: Session,
    user_id: UUID,
    task_id: UUID,
    task_title: str
) -> Notification | None:
    """Create a notification when a task is deleted."""
    return create_notification(
        session=session,
        user_id=user_id,
        title="Task Deleted",
        message=f'Task "{task_title}" has been deleted',
        notification_type=NotificationType.TASK_DELETED,
        task_id=task_id
    )


def create_task_completed_notification(
    session: Session,
    user_id: UUID,
    task_id: UUID,
    task_title: str
) -> Notification | None:
    """Create a notification when a task is completed."""
    return create_notification(
        session=session,
        user_id=user_id,
        title="Task Completed",
        message=f'Task "{task_title}" has been completed',
        notification_type=NotificationType.TASK_COMPLETED,
        task_id=task_id
    )
