"""Notifications API endpoints."""
from fastapi import APIRouter, Depends, HTTPException, status, Header, Query
from sqlmodel import Session, select, func
from typing import Optional
from uuid import UUID

from config.database import get_session
from core.security import decode_access_token
from models.notification_models import Notification

router = APIRouter(prefix="/notifications", tags=["notifications"])


@router.get("")
def get_notifications(
    limit: int = Query(10, ge=1, le=100),
    offset: int = Query(0, ge=0),
    unread_only: bool = Query(False),
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """Get list of notifications for the current user."""
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Missing or invalid authorization header",
        )

    # Extract token from "Bearer <token>"
    token = authorization.split(" ")[1]

    # Decode and validate token
    payload = decode_access_token(token)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
        )

    # Get user_id from token
    user_id_str = payload.get("sub")
    if not user_id_str:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token payload",
        )

    try:
        user_id = UUID(user_id_str)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid user ID in token",
        )

    # Build query for notifications
    statement = select(Notification).where(Notification.user_id == user_id)

    # Filter by read status if requested
    if unread_only:
        statement = statement.where(Notification.is_read == False)

    # Order by created_at descending (newest first)
    statement = statement.order_by(Notification.created_at.desc())

    # Get total count
    count_statement = select(func.count()).select_from(
        select(Notification).where(Notification.user_id == user_id).subquery()
    )
    if unread_only:
        count_statement = select(func.count()).select_from(
            select(Notification).where(
                Notification.user_id == user_id,
                Notification.is_read == False
            ).subquery()
        )
    total = session.exec(count_statement).one() or 0

    # Get unread count
    unread_count_statement = select(func.count()).select_from(
        select(Notification).where(
            Notification.user_id == user_id,
            Notification.is_read == False
        ).subquery()
    )
    unread_count = session.exec(unread_count_statement).one() or 0

    # Apply pagination
    statement = statement.offset(offset).limit(limit)

    # Execute query
    notifications = session.exec(statement).all()

    return {
        "notifications": notifications,
        "unread_count": unread_count,
        "total": total,
        "limit": limit,
        "offset": offset
    }


@router.get("/unread-count")
def get_unread_count(
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """Get count of unread notifications for the current user."""
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Missing or invalid authorization header",
        )

    # Extract token from "Bearer <token>"
    token = authorization.split(" ")[1]

    # Decode and validate token
    payload = decode_access_token(token)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
        )

    # Get user_id from token
    user_id_str = payload.get("sub")
    if not user_id_str:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token payload",
        )

    try:
        user_id = UUID(user_id_str)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid user ID in token",
        )

    # Count unread notifications for the user
    count_statement = select(func.count()).select_from(
        select(Notification).where(
            Notification.user_id == user_id,
            Notification.is_read == False
        ).subquery()
    )
    count = session.exec(count_statement).one() or 0

    return {
        "unread_count": count,
        "user_id": str(user_id)
    }


@router.post("/mark-all-read")
def mark_all_as_read(
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """Mark all notifications as read for the current user."""
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Missing or invalid authorization header",
        )

    # Extract token from "Bearer <token>"
    token = authorization.split(" ")[1]

    # Decode and validate token
    payload = decode_access_token(token)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
        )

    # Get user_id from token
    user_id_str = payload.get("sub")
    if not user_id_str:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token payload",
        )

    try:
        user_id = UUID(user_id_str)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid user ID in token",
        )

    # Get all unread notifications for the user
    statement = select(Notification).where(
        Notification.user_id == user_id,
        Notification.is_read == False
    )
    notifications = session.exec(statement).all()

    # Mark all as read
    marked_count = 0
    for notification in notifications:
        notification.is_read = True
        marked_count += 1

    session.commit()

    return {
        "marked_count": marked_count,
        "user_id": str(user_id)
    }


@router.delete("")
def clear_all_notifications(
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """Delete all notifications for the current user."""
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Missing or invalid authorization header",
        )

    # Extract token from "Bearer <token>"
    token = authorization.split(" ")[1]

    # Decode and validate token
    payload = decode_access_token(token)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
        )

    # Get user_id from token
    user_id_str = payload.get("sub")
    if not user_id_str:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token payload",
        )

    try:
        user_id = UUID(user_id_str)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid user ID in token",
        )

    # Get all notifications for the user
    statement = select(Notification).where(Notification.user_id == user_id)
    notifications = session.exec(statement).all()

    # Delete all notifications
    deleted_count = 0
    for notification in notifications:
        session.delete(notification)
        deleted_count += 1

    session.commit()

    return {
        "deleted_count": deleted_count,
        "user_id": str(user_id)
    }


@router.patch("/{notification_id}/read")
def mark_notification_as_read(
    notification_id: UUID,
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """Mark a single notification as read."""
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Missing or invalid authorization header",
        )

    # Extract token from "Bearer <token>"
    token = authorization.split(" ")[1]

    # Decode and validate token
    payload = decode_access_token(token)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
        )

    # Get user_id from token
    user_id_str = payload.get("sub")
    if not user_id_str:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token payload",
        )

    try:
        user_id = UUID(user_id_str)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid user ID in token",
        )

    # Get the notification
    statement = select(Notification).where(
        Notification.id == notification_id,
        Notification.user_id == user_id
    )
    notification = session.exec(statement).first()

    if not notification:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Notification not found",
        )

    # Mark as read
    notification.is_read = True
    session.commit()
    session.refresh(notification)

    return {
        "id": str(notification.id),
        "is_read": notification.is_read,
        "user_id": str(user_id)
    }


@router.delete("/{notification_id}")
def delete_notification(
    notification_id: UUID,
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """Delete a single notification."""
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Missing or invalid authorization header",
        )

    # Extract token from "Bearer <token>"
    token = authorization.split(" ")[1]

    # Decode and validate token
    payload = decode_access_token(token)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
        )

    # Get user_id from token
    user_id_str = payload.get("sub")
    if not user_id_str:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token payload",
        )

    try:
        user_id = UUID(user_id_str)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid user ID in token",
        )

    # Get the notification
    statement = select(Notification).where(
        Notification.id == notification_id,
        Notification.user_id == user_id
    )
    notification = session.exec(statement).first()

    if not notification:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Notification not found",
        )

    # Delete the notification
    session.delete(notification)
    session.commit()

    return {
        "id": str(notification_id),
        "deleted": True,
        "user_id": str(user_id)
    }
