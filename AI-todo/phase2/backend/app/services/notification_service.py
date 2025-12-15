"""Notification service for business logic."""

from datetime import date
from uuid import UUID

from sqlmodel import func, select
from sqlmodel.ext.asyncio.session import AsyncSession

from app.models.notification import Notification

# Maximum notifications per user (cleanup triggered on create)
NOTIFICATION_LIMIT = 100


async def get_notifications(
    db: AsyncSession,
    user_id: UUID,
    limit: int = 20,
    offset: int = 0,
    unread_only: bool = False,
) -> tuple[list[Notification], int, int]:
    """Get paginated notifications for a user.

    Args:
        db: Async database session.
        user_id: User ID to get notifications for.
        limit: Maximum notifications to return.
        offset: Pagination offset.
        unread_only: If True, only return unread notifications.

    Returns:
        Tuple of (notifications list, unread count, total count).
    """
    # Build query
    query = select(Notification).where(Notification.user_id == user_id)
    if unread_only:
        query = query.where(Notification.is_read == False)  # noqa: E712
    query = query.order_by(Notification.created_at.desc()).offset(offset).limit(limit)

    result = await db.exec(query)
    notifications = list(result.all())

    # Get counts
    unread_count = await get_unread_count(db, user_id)

    total_result = await db.exec(
        select(func.count())
        .select_from(Notification)
        .where(Notification.user_id == user_id)
    )
    total = total_result.first() or 0

    return notifications, unread_count, total


async def get_unread_count(db: AsyncSession, user_id: UUID) -> int:
    """Get unread notification count for a user.

    Args:
        db: Async database session.
        user_id: User ID to get count for.

    Returns:
        Number of unread notifications.
    """
    result = await db.exec(
        select(func.count())
        .select_from(Notification)
        .where(Notification.user_id == user_id, Notification.is_read == False)  # noqa: E712
    )
    return result.first() or 0


async def mark_as_read(
    db: AsyncSession,
    notification_id: UUID,
    user_id: UUID,
) -> Notification | None:
    """Mark a single notification as read.

    Args:
        db: Async database session.
        notification_id: ID of notification to mark.
        user_id: User ID (for ownership verification).

    Returns:
        Updated notification or None if not found/not owned.
    """
    result = await db.exec(
        select(Notification).where(
            Notification.id == notification_id,
            Notification.user_id == user_id,
        )
    )
    notification = result.first()

    if notification:
        notification.is_read = True
        db.add(notification)
        await db.commit()
        await db.refresh(notification)

    return notification


async def mark_all_as_read(db: AsyncSession, user_id: UUID) -> int:
    """Mark all notifications as read for a user.

    Args:
        db: Async database session.
        user_id: User ID to mark notifications for.

    Returns:
        Number of notifications marked as read.
    """
    result = await db.exec(
        select(Notification).where(
            Notification.user_id == user_id,
            Notification.is_read == False,  # noqa: E712
        )
    )
    notifications = list(result.all())
    count = len(notifications)

    for notification in notifications:
        notification.is_read = True
        db.add(notification)

    await db.commit()
    return count


async def delete_notification(
    db: AsyncSession,
    notification_id: UUID,
    user_id: UUID,
) -> bool:
    """Delete a single notification.

    Args:
        db: Async database session.
        notification_id: ID of notification to delete.
        user_id: User ID (for ownership verification).

    Returns:
        True if deleted, False if not found/not owned.
    """
    result = await db.exec(
        select(Notification).where(
            Notification.id == notification_id,
            Notification.user_id == user_id,
        )
    )
    notification = result.first()

    if notification:
        await db.delete(notification)
        await db.commit()
        return True

    return False


async def clear_all_notifications(db: AsyncSession, user_id: UUID) -> int:
    """Delete all notifications for a user.

    Args:
        db: Async database session.
        user_id: User ID to clear notifications for.

    Returns:
        Number of notifications deleted.
    """
    result = await db.exec(
        select(Notification).where(Notification.user_id == user_id)
    )
    notifications = list(result.all())
    count = len(notifications)

    for notification in notifications:
        await db.delete(notification)

    await db.commit()
    return count


async def create_notification(
    db: AsyncSession,
    user_id: UUID,
    type: str,
    title: str,
    message: str,
    action_url: str | None = None,
) -> Notification:
    """Create a new notification for a user.

    Automatically cleans up old notifications if count exceeds limit.

    Args:
        db: Async database session.
        user_id: User ID to create notification for.
        type: Notification type (task_due, task_overdue, task_completed, welcome, system).
        title: Short title.
        message: Full message text.
        action_url: Optional deep link URL.

    Returns:
        Created notification.
    """
    notification = Notification(
        user_id=user_id,
        type=type,
        title=title,
        message=message,
        action_url=action_url,
    )
    db.add(notification)

    # Check count and cleanup if over limit
    count_result = await db.exec(
        select(func.count())
        .select_from(Notification)
        .where(Notification.user_id == user_id)
    )
    count = (count_result.first() or 0) + 1  # +1 for the new notification

    if count > NOTIFICATION_LIMIT:
        # Delete oldest notifications to stay under limit
        excess = count - NOTIFICATION_LIMIT
        old_notifications_result = await db.exec(
            select(Notification)
            .where(Notification.user_id == user_id)
            .order_by(Notification.created_at.asc())
            .limit(excess)
        )
        for old_notification in old_notifications_result.all():
            await db.delete(old_notification)

    await db.commit()
    await db.refresh(notification)
    return notification


async def notify_task_created(
    db: AsyncSession,
    user_id: UUID,
    task_title: str,
    task_id: UUID,
    due_date: date | None = None,
) -> Notification:
    """Create notification when a task is created with a due date.

    Args:
        db: Async database session.
        user_id: User ID to notify.
        task_title: Title of the created task.
        task_id: ID of the created task.
        due_date: Optional due date of the task.

    Returns:
        Created notification.
    """
    if due_date:
        message = f"Task '{task_title}' created with due date {due_date.strftime('%b %d, %Y')}"
    else:
        message = f"Task '{task_title}' has been created"

    return await create_notification(
        db=db,
        user_id=user_id,
        type="task_created",
        title="Task Created",
        message=message,
        action_url=f"/tasks?highlight={task_id}",
    )


async def notify_task_due_soon(
    db: AsyncSession,
    user_id: UUID,
    task_title: str,
    task_id: UUID,
    due_date: date,
) -> Notification:
    """Create notification when a task is due soon (within 24 hours).

    Args:
        db: Async database session.
        user_id: User ID to notify.
        task_title: Title of the task.
        task_id: ID of the task.
        due_date: Due date of the task.

    Returns:
        Created notification.
    """
    return await create_notification(
        db=db,
        user_id=user_id,
        type="task_due",
        title="Task Due Soon",
        message=f"'{task_title}' is due {due_date.strftime('%b %d, %Y')}",
        action_url=f"/tasks?highlight={task_id}",
    )


async def notify_task_overdue(
    db: AsyncSession,
    user_id: UUID,
    task_title: str,
    task_id: UUID,
    due_date: date,
) -> Notification:
    """Create notification when a task becomes overdue.

    Args:
        db: Async database session.
        user_id: User ID to notify.
        task_title: Title of the task.
        task_id: ID of the task.
        due_date: Due date of the task.

    Returns:
        Created notification.
    """
    return await create_notification(
        db=db,
        user_id=user_id,
        type="task_overdue",
        title="Task Overdue",
        message=f"'{task_title}' was due {due_date.strftime('%b %d, %Y')}",
        action_url=f"/tasks?highlight={task_id}",
    )


async def notify_task_completed(
    db: AsyncSession,
    user_id: UUID,
    task_title: str,
    task_id: UUID,
) -> Notification:
    """Create notification when a task is completed.

    Args:
        db: Async database session.
        user_id: User ID to notify.
        task_title: Title of the completed task.
        task_id: ID of the completed task.

    Returns:
        Created notification.
    """
    return await create_notification(
        db=db,
        user_id=user_id,
        type="task_completed",
        title="Task Completed",
        message=f"Great job! '{task_title}' has been completed",
        action_url=f"/tasks?highlight={task_id}",
    )


async def notify_task_deleted(
    db: AsyncSession,
    user_id: UUID,
    task_title: str,
) -> Notification:
    """Create notification when a task is deleted.

    Args:
        db: Async database session.
        user_id: User ID to notify.
        task_title: Title of the deleted task.

    Returns:
        Created notification.
    """
    return await create_notification(
        db=db,
        user_id=user_id,
        type="task_deleted",
        title="Task Deleted",
        message=f"'{task_title}' has been removed",
        action_url="/tasks",
    )
