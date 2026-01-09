from fastapi import APIRouter, Depends, HTTPException, status, Header
from sqlmodel import Session, select
from typing import List, Optional
from datetime import datetime, timedelta, date
from uuid import UUID
import json

from models.task_models import (
    Task,
    Reminder,
    ReminderCreate,
    ReminderUpdate,
    ReminderRead,
    TaskSearchRequest,
    TaskSearchResponse
)
from config.database import get_session
from core.security import decode_access_token


router = APIRouter(prefix="", tags=["reminders"])


def get_authenticated_user_id(authorization: Optional[str]) -> UUID:
    """Helper function to extract and validate user_id from JWT token."""
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
        return UUID(user_id_str)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid user ID in token"
        )


@router.put("/tasks/{task_id}/due-date")
def set_task_due_date(
    task_id: UUID,
    due_date: date,
    reminder_times: List[datetime] = None,
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """
    Set a due date for a task and optionally schedule reminder times.
    Requires authentication - users can only modify their own tasks.
    """
    try:
        authenticated_user_id = get_authenticated_user_id(authorization)

        # Get the task
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

        # Update the task with the due date
        task.due_date = due_date
        if reminder_times:
            task.reminder_times = reminder_times

        session.add(task)
        session.commit()
        session.refresh(task)

        # Create reminder records if reminder times are provided
        if reminder_times:
            for reminder_time in reminder_times:
                reminder = Reminder(
                    user_id=task.user_id,
                    task_id=task.id,
                    reminder_time=reminder_time,
                    notification_method="push"  # Default method
                )
                session.add(reminder)

            session.commit()

        return {
            "task_id": task.id,
            "due_date": task.due_date,
            "reminder_times": task.reminder_times,
            "message": "Due date and reminders set successfully"
        }

    except HTTPException:
        raise
    except Exception as e:
        session.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error setting due date: {str(e)}"
        )


@router.put("/tasks/{task_id}/reminders")
def set_task_reminders(
    task_id: UUID,
    reminder_times: List[datetime],
    notification_method: str = "push",
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """
    Set reminder times for a task.
    Requires authentication - users can only modify their own tasks.
    """
    try:
        authenticated_user_id = get_authenticated_user_id(authorization)

        # Get the task to verify it exists
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

        # Update the task with reminder times
        task.reminder_times = reminder_times
        session.add(task)

        # Create reminder records
        for reminder_time in reminder_times:
            reminder = Reminder(
                user_id=task.user_id,
                task_id=task.id,
                reminder_time=reminder_time,
                notification_method=notification_method
            )
            session.add(reminder)

        session.commit()

        return {
            "task_id": task.id,
            "reminder_times": reminder_times,
            "message": "Reminders set successfully"
        }

    except HTTPException:
        raise
    except Exception as e:
        session.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error setting reminders: {str(e)}"
        )


@router.get("/users/{user_id}/due-soon", response_model=List[Task])
def get_due_soon_tasks(
    user_id: str,
    within_hours: int = 24,
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """
    Get tasks that are due within the specified number of hours.
    Use 'me' as user_id to get tasks for the authenticated user.
    Requires authentication.
    """
    try:
        authenticated_user_id = get_authenticated_user_id(authorization)

        # Handle "me" as a special case for authenticated user
        if user_id == "me":
            resolved_user_id = authenticated_user_id
        else:
            try:
                resolved_user_id = UUID(user_id)
            except ValueError:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="Invalid user ID format"
                )

            # Verify the user can only access their own tasks
            if resolved_user_id != authenticated_user_id:
                raise HTTPException(
                    status_code=status.HTTP_403_FORBIDDEN,
                    detail="Access denied"
                )

        # Calculate the time threshold
        time_threshold = datetime.utcnow() + timedelta(hours=within_hours)

        # Query for tasks that are not completed and are due within the threshold
        statement = select(Task).where(
            Task.user_id == resolved_user_id,
            Task.status != "completed",
            Task.due_date <= time_threshold.date(),
            Task.due_date >= datetime.utcnow().date()
        ).order_by(Task.due_date.asc())

        due_soon_tasks = session.exec(statement).all()
        return due_soon_tasks

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving due soon tasks: {str(e)}"
        )


@router.get("/users/{user_id}/upcoming-reminders", response_model=List[ReminderRead])
def get_upcoming_reminders(
    user_id: str,
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """
    Get upcoming reminders for a user that have not been sent yet.
    Use 'me' as user_id to get reminders for the authenticated user.
    Requires authentication.
    """
    try:
        authenticated_user_id = get_authenticated_user_id(authorization)

        # Handle "me" as a special case for authenticated user
        if user_id == "me":
            resolved_user_id = authenticated_user_id
        else:
            try:
                resolved_user_id = UUID(user_id)
            except ValueError:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="Invalid user ID format"
                )

            # Verify the user can only access their own reminders
            if resolved_user_id != authenticated_user_id:
                raise HTTPException(
                    status_code=status.HTTP_403_FORBIDDEN,
                    detail="Access denied"
                )

        statement = select(Reminder).where(
            Reminder.user_id == resolved_user_id,
            Reminder.is_sent == False,
            Reminder.reminder_time >= datetime.utcnow()
        ).order_by(Reminder.reminder_time.asc())

        upcoming_reminders = session.exec(statement).all()
        return upcoming_reminders

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving upcoming reminders: {str(e)}"
        )


@router.post("/reminders/{reminder_id}/send")
def send_reminder(
    reminder_id: UUID,
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """
    Mark a reminder as sent. This would typically be called by a notification service.
    Requires authentication - users can only mark their own reminders as sent.
    """
    try:
        authenticated_user_id = get_authenticated_user_id(authorization)

        reminder = session.get(Reminder, reminder_id)
        if not reminder:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Reminder not found"
            )

        # Verify the reminder belongs to the authenticated user
        if reminder.user_id != authenticated_user_id:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Access denied"
            )

        if reminder.is_sent:
            return {"message": "Reminder already sent"}

        # Update the reminder as sent
        reminder.is_sent = True
        reminder.sent_at = datetime.utcnow()
        session.add(reminder)
        session.commit()

        # Also update the task to mark that a reminder was sent
        task = session.get(Task, reminder.task_id)
        if task:
            task.is_reminder_sent = True
            session.add(task)
            session.commit()

        return {"message": "Reminder sent successfully"}

    except HTTPException:
        raise
    except Exception as e:
        session.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error sending reminder: {str(e)}"
        )


@router.post("/tasks/{task_id}/snooze-reminder")
def snooze_reminder(
    task_id: UUID,
    snooze_minutes: int = 30,
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """
    Snooze a reminder for the specified number of minutes.
    Requires authentication - users can only snooze their own task reminders.
    """
    try:
        authenticated_user_id = get_authenticated_user_id(authorization)

        # Get the task
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

        # Find the next upcoming reminder for this task that hasn't been sent
        statement = select(Reminder).where(
            Reminder.task_id == task_id,
            Reminder.is_sent == False,
            Reminder.reminder_time >= datetime.utcnow()
        ).order_by(Reminder.reminder_time.asc()).limit(1)

        next_reminder = session.exec(statement).first()
        if not next_reminder:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="No upcoming reminder found for this task"
            )

        # Update the reminder time to be snoozed
        new_reminder_time = datetime.utcnow() + timedelta(minutes=snooze_minutes)
        next_reminder.reminder_time = new_reminder_time
        session.add(next_reminder)
        session.commit()

        return {
            "message": f"Reminder snoozed for {snooze_minutes} minutes",
            "new_reminder_time": new_reminder_time
        }

    except HTTPException:
        raise
    except Exception as e:
        session.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error snoozing reminder: {str(e)}"
        )


@router.get("/users/{user_id}/overdue-tasks", response_model=List[Task])
def get_overdue_tasks(
    user_id: str,
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """
    Get tasks that are overdue (due date is in the past and task is not completed).
    Use 'me' as user_id to get tasks for the authenticated user.
    Requires authentication.
    """
    try:
        authenticated_user_id = get_authenticated_user_id(authorization)

        # Handle "me" as a special case for authenticated user
        if user_id == "me":
            resolved_user_id = authenticated_user_id
        else:
            try:
                resolved_user_id = UUID(user_id)
            except ValueError:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="Invalid user ID format"
                )

            # Verify the user can only access their own tasks
            if resolved_user_id != authenticated_user_id:
                raise HTTPException(
                    status_code=status.HTTP_403_FORBIDDEN,
                    detail="Access denied"
                )

        statement = select(Task).where(
            Task.user_id == resolved_user_id,
            Task.status != "completed",
            Task.due_date < datetime.utcnow().date()
        ).order_by(Task.due_date.asc())

        overdue_tasks = session.exec(statement).all()
        return overdue_tasks

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving overdue tasks: {str(e)}"
        )