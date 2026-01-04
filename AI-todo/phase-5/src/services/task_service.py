from sqlmodel import Session
from datetime import datetime, timedelta
from typing import Optional
import logging
from ..models.task_models import Task, RecurringTaskPattern, TaskCreate


def create_task_from_recurring_pattern(session: Session, pattern: RecurringTaskPattern) -> Task:
    """
    Create a new task instance based on a recurring task pattern.
    This function generates the next occurrence based on the pattern.
    """
    # Create a new task based on the pattern
    new_task = Task(
        title=pattern.base_task_title,
        description=pattern.base_task_description,
        user_id=pattern.user_id,
        completed=False,
        priority="medium",  # Default priority, could be from pattern
        tags=[],  # Default tags, could be from pattern
        due_date=None,  # No due date by default, could be calculated from pattern
        is_recurring=True,
        recurring_pattern_id=pattern.id,
        created_by=pattern.user_id,
        updated_by=pattern.user_id
    )

    session.add(new_task)
    session.commit()
    session.refresh(new_task)

    return new_task


def calculate_next_occurrence(pattern: RecurringTaskPattern, current_date: datetime = None) -> Optional[datetime]:
    """
    Calculate the next occurrence date based on the recurring pattern.
    """
    if current_date is None:
        current_date = datetime.utcnow()

    # Set to the start of the day for consistent calculations
    current_date = current_date.replace(hour=0, minute=0, second=0, microsecond=0)

    if pattern.pattern_type == "daily":
        next_date = current_date + timedelta(days=pattern.interval)
    elif pattern.pattern_type == "weekly":
        # Calculate next occurrence based on weekdays
        next_date = current_date + timedelta(weeks=pattern.interval if pattern.interval > 0 else 1)

        # If weekdays are specified, find the next occurrence on those days
        if pattern.weekdays:
            # Find the next occurrence on one of the specified weekdays
            next_date = find_next_weekday(current_date, pattern.weekdays)
    elif pattern.pattern_type == "monthly":
        # Calculate next month occurrence
        next_date = add_months(current_date, pattern.interval if pattern.interval > 0 else 1)

        # If days of month are specified, find the next occurrence on those days
        if pattern.days_of_month:
            next_date = find_next_day_of_month(current_date, pattern.days_of_month)
    elif pattern.pattern_type == "yearly":
        # Calculate next year occurrence
        next_date = add_years(current_date, pattern.interval if pattern.interval > 0 else 1)
    else:
        # For custom patterns, we might need more complex logic
        # For now, default to the same day next month
        next_date = add_months(current_date, 1)

    # Check if the calculated date is beyond the pattern's end date
    if pattern.end_date and next_date > pattern.end_date:
        return None

    return next_date


def find_next_weekday(current_date: datetime, weekdays: list) -> datetime:
    """
    Find the next occurrence on one of the specified weekdays.
    Weekdays: 0=Sunday, 1=Monday, ..., 6=Saturday
    """
    current_weekday = current_date.weekday()  # Monday=0, Sunday=6 in Python
    # Convert to our format where Sunday=0
    current_weekday = (current_weekday + 1) % 7

    # Sort the target weekdays
    weekdays = sorted(weekdays)

    # Find the next occurrence
    for day in weekdays:
        if day > current_weekday:
            # Next occurrence is later this week
            days_ahead = day - current_weekday
            return current_date + timedelta(days=days_ahead)

    # If no day later this week, go to next week
    days_ahead = 7 - current_weekday + weekdays[0]
    return current_date + timedelta(days=days_ahead)


def find_next_day_of_month(current_date: datetime, days_of_month: list) -> datetime:
    """
    Find the next occurrence on one of the specified days of the month.
    """
    days_of_month = sorted(days_of_month)

    # Get the current day of month
    current_day = current_date.day

    # Find the next day in the list
    for day in days_of_month:
        if day >= current_day:
            # Same month
            try:
                return current_date.replace(day=day)
            except ValueError:
                # Day doesn't exist in this month (e.g., Feb 30th)
                # Go to next month
                next_month = add_months(current_date.replace(day=1), 1)
                return find_next_day_of_month(next_month, days_of_month)

    # If no day left in current month, go to next month
    next_month = add_months(current_date.replace(day=1), 1)
    return find_next_day_of_month(next_month, days_of_month)


def add_months(source_date: datetime, months: int) -> datetime:
    """
    Add months to a date, handling month-end edge cases.
    """
    month = source_date.month - 1 + months
    year = source_date.year + month // 12
    month = month % 12 + 1

    # Handle day overflow (e.g., Jan 31 + 1 month should be Feb 28/29, not Mar)
    day = min(source_date.day, [31,
        29 if year % 4 == 0 and (year % 100 != 0 or year % 400 == 0) else 28,
        31, 30, 31, 30, 31, 31, 30, 31, 30, 31][month-1])

    return source_date.replace(year=year, month=month, day=day)


def add_years(source_date: datetime, years: int) -> datetime:
    """
    Add years to a date, handling leap year edge cases.
    """
    try:
        return source_date.replace(year=source_date.year + years)
    except ValueError:
        # Handle Feb 29 on non-leap years
        return source_date.replace(year=source_date.year + years, day=28)


def process_completed_recurring_task(session: Session, task_id: int):
    """
    Process a completed recurring task to generate the next occurrence.
    """
    # Get the completed task
    completed_task = session.get(Task, task_id)
    if not completed_task or not completed_task.is_recurring or not completed_task.recurring_pattern_id:
        return None

    # Get the recurring pattern
    pattern = session.get(RecurringTaskPattern, completed_task.recurring_pattern_id)
    if not pattern:
        return None

    # Calculate the next occurrence date
    next_occurrence_date = calculate_next_occurrence(pattern)
    if not next_occurrence_date:
        # Pattern has ended, no more occurrences
        return None

    # Create a new task based on the pattern
    new_task = Task(
        title=pattern.base_task_title,
        description=pattern.base_task_description,
        user_id=pattern.user_id,
        completed=False,
        priority=completed_task.priority,  # Inherit priority from completed task
        tags=completed_task.tags,  # Inherit tags from completed task
        due_date=next_occurrence_date,  # Set due date to next occurrence
        is_recurring=True,
        recurring_pattern_id=pattern.id,
        parent_task_id=completed_task.id,  # Link to the completed task
        created_by=pattern.user_id,
        updated_by=pattern.user_id
    )

    session.add(new_task)
    session.commit()
    session.refresh(new_task)

    # Update the completed task to link to the next occurrence
    completed_task.next_occurrence_id = new_task.id
    session.add(completed_task)
    session.commit()

    return new_task


def get_due_tasks_for_user(session: Session, user_id: str, within_hours: int = 24):
    """
    Get tasks that are due within the specified number of hours for a user.
    """
    from sqlmodel import select
    from datetime import datetime, timedelta

    time_threshold = datetime.utcnow() + timedelta(hours=within_hours)

    statement = select(Task).where(
        Task.user_id == user_id,
        Task.completed == False,
        Task.due_date <= time_threshold,
        Task.due_date >= datetime.utcnow()
    ).order_by(Task.due_date.asc())

    due_tasks = session.exec(statement).all()
    return due_tasks


def get_overdue_tasks_for_user(session: Session, user_id: str):
    """
    Get tasks that are overdue for a user.
    """
    from sqlmodel import select
    from datetime import datetime

    statement = select(Task).where(
        Task.user_id == user_id,
        Task.completed == False,
        Task.due_date < datetime.utcnow()
    ).order_by(Task.due_date.asc())

    overdue_tasks = session.exec(statement).all()
    return overdue_tasks


def get_tasks_by_priority(session: Session, user_id: str, priority: str):
    """
    Get tasks for a user filtered by priority level.
    """
    from sqlmodel import select

    statement = select(Task).where(
        Task.user_id == user_id,
        Task.priority == priority
    ).order_by(Task.created_at.desc())

    tasks = session.exec(statement).all()
    return tasks


def get_tasks_by_tags(session: Session, user_id: str, tags: list):
    """
    Get tasks for a user that have any of the specified tags.
    """
    from sqlmodel import select
    from sqlalchemy import func

    statement = select(Task).where(
        Task.user_id == user_id
    )

    # Filter by tags
    for tag in tags:
        statement = statement.where(func.array_to_string(Task.tags, ',').contains(tag))

    tasks = session.exec(statement).all()
    return tasks