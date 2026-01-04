from fastapi import APIRouter, Depends, HTTPException, status
from sqlmodel import Session, select
from typing import List
from datetime import datetime, timedelta, date
from uuid import UUID
import json

from ..models.task_models import (
    Task,
    Reminder,
    ReminderCreate,
    ReminderUpdate,
    ReminderRead,
    TaskSearchRequest,
    TaskSearchResponse
)
from ..config.database import get_session


router = APIRouter(prefix="", tags=["reminders"])


@router.put("/tasks/{task_id}/due-date")
def set_task_due_date(
    task_id: UUID,
    due_date: date,
    reminder_times: List[datetime] = None,
    session: Session = Depends(get_session)
):
    """
    Set a due date for a task and optionally schedule reminder times.
    """
    try:
        # Get the task
        task = session.get(Task, task_id)
        if not task:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Task not found"
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
    session: Session = Depends(get_session)
):
    """
    Set reminder times for a task.
    """
    try:
        # Get the task to verify it exists
        task = session.get(Task, task_id)
        if not task:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Task not found"
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
    user_id: UUID,
    within_hours: int = 24,
    session: Session = Depends(get_session)
):
    """
    Get tasks that are due within the specified number of hours.
    """
    try:
        # Calculate the time threshold
        time_threshold = datetime.utcnow() + timedelta(hours=within_hours)

        # Query for tasks that are not completed and are due within the threshold
        statement = select(Task).where(
            Task.user_id == user_id,
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
    user_id: UUID,
    session: Session = Depends(get_session)
):
    """
    Get upcoming reminders for a user that have not been sent yet.
    """
    try:
        statement = select(Reminder).where(
            Reminder.user_id == user_id,
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
    session: Session = Depends(get_session)
):
    """
    Mark a reminder as sent. This would typically be called by a notification service.
    """
    try:
        reminder = session.get(Reminder, reminder_id)
        if not reminder:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Reminder not found"
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
    session: Session = Depends(get_session)
):
    """
    Snooze a reminder for the specified number of minutes.
    """
    try:
        # Get the task
        task = session.get(Task, task_id)
        if not task:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Task not found"
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
    user_id: UUID,
    session: Session = Depends(get_session)
):
    """
    Get tasks that are overdue (due date is in the past and task is not completed).
    """
    try:
        statement = select(Task).where(
            Task.user_id == user_id,
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