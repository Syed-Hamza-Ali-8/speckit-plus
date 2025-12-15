"""Notification API endpoints."""

from uuid import UUID

from fastapi import APIRouter, HTTPException, Query, status

from app.api.deps import CurrentUser, DbSession
from app.schemas.notification import (
    NotificationClearAllResponse,
    NotificationDeleteResponse,
    NotificationListResponse,
    NotificationMarkAllReadResponse,
    NotificationMarkReadResponse,
    NotificationResponse,
    UnreadCountResponse,
)
from app.services import notification_service

router = APIRouter(prefix="/notifications", tags=["notifications"])


@router.get("", response_model=NotificationListResponse)
async def get_notifications(
    current_user: CurrentUser,
    db: DbSession,
    limit: int = Query(20, ge=1, le=100, description="Max notifications per page"),
    offset: int = Query(0, ge=0, description="Pagination offset"),
    unread_only: bool = Query(False, description="Filter to unread only"),
) -> NotificationListResponse:
    """Get paginated notifications for the current user.

    Returns notifications sorted by created_at descending (newest first).
    """
    notifications, unread_count, total = await notification_service.get_notifications(
        db, current_user.id, limit, offset, unread_only
    )
    return NotificationListResponse(
        notifications=[NotificationResponse.model_validate(n) for n in notifications],
        unread_count=unread_count,
        total=total,
    )


@router.patch("/{notification_id}/read", response_model=NotificationMarkReadResponse)
async def mark_notification_read(
    notification_id: UUID,
    current_user: CurrentUser,
    db: DbSession,
) -> NotificationMarkReadResponse:
    """Mark a single notification as read.

    Returns 404 if notification not found or not owned by current user.
    """
    notification = await notification_service.mark_as_read(
        db, notification_id, current_user.id
    )
    if not notification:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Notification not found",
        )
    return NotificationMarkReadResponse(id=notification.id, is_read=True)


@router.post("/mark-all-read", response_model=NotificationMarkAllReadResponse)
async def mark_all_notifications_read(
    current_user: CurrentUser,
    db: DbSession,
) -> NotificationMarkAllReadResponse:
    """Mark all notifications as read for the current user."""
    count = await notification_service.mark_all_as_read(db, current_user.id)
    return NotificationMarkAllReadResponse(marked_count=count)


@router.get("/unread-count", response_model=UnreadCountResponse)
async def get_unread_count(
    current_user: CurrentUser,
    db: DbSession,
) -> UnreadCountResponse:
    """Get unread notification count for the current user.

    This is a lightweight endpoint for badge polling.
    """
    count = await notification_service.get_unread_count(db, current_user.id)
    return UnreadCountResponse(unread_count=count)


@router.delete("/{notification_id}", response_model=NotificationDeleteResponse)
async def delete_notification(
    notification_id: UUID,
    current_user: CurrentUser,
    db: DbSession,
) -> NotificationDeleteResponse:
    """Delete a single notification.

    Returns 404 if notification not found or not owned by current user.
    """
    deleted = await notification_service.delete_notification(
        db, notification_id, current_user.id
    )
    if not deleted:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Notification not found",
        )
    return NotificationDeleteResponse(id=notification_id, deleted=True)


@router.delete("", response_model=NotificationClearAllResponse)
async def clear_all_notifications(
    current_user: CurrentUser,
    db: DbSession,
) -> NotificationClearAllResponse:
    """Delete all notifications for the current user."""
    count = await notification_service.clear_all_notifications(db, current_user.id)
    return NotificationClearAllResponse(deleted_count=count)
