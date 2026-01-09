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

    # Apply pagination
    statement = statement.offset(offset).limit(limit)

    # Execute query
    notifications = session.exec(statement).all()

    return {
        "items": notifications,
        "total": total,
        "limit": limit,
        "offset": offset,
        "unread_only": unread_only
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
        "count": count,
        "user_id": str(user_id)
    }
