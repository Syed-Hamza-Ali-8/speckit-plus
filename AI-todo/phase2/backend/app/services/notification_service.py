"""Notification service for business logic."""

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
