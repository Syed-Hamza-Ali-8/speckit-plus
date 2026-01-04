from fastapi import APIRouter, Depends, HTTPException, status
from sqlmodel import Session, select
from typing import List
from datetime import date
from uuid import UUID

from ..models.task_models import (
    RecurringTaskPattern,
    RecurringTaskPatternCreate,
    RecurringTaskPatternUpdate,
    RecurringTaskPatternRead,
    Task,
    TaskCreate
)
from ..config.database import get_session
from ..services.task_service import create_task_from_recurring_pattern


router = APIRouter(prefix="", tags=["recurring-tasks"])


@router.post("/recurring-tasks", response_model=RecurringTaskPatternRead)
def create_recurring_task_pattern(
    recurring_pattern: RecurringTaskPatternCreate,
    session: Session = Depends(get_session)
):
    """
    Create a recurring task pattern that will generate tasks based on the specified schedule.
    """
    try:
        # Validate the recurrence pattern
        if recurring_pattern.end_date and recurring_pattern.start_date > recurring_pattern.end_date:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="End date must be after start date"
            )

        # Validate weekdays for weekly patterns
        if recurring_pattern.pattern_type == "weekly" and recurring_pattern.weekdays:
            for day in recurring_pattern.weekdays:
                if not 0 <= day <= 6:  # 0=Sunday, 6=Saturday
                    raise HTTPException(
                        status_code=status.HTTP_400_BAD_REQUEST,
                        detail="Weekdays must be between 0 (Sunday) and 6 (Saturday)"
                    )

        # Validate days of month for monthly patterns
        if recurring_pattern.pattern_type == "monthly" and recurring_pattern.days_of_month:
            for day in recurring_pattern.days_of_month:
                if not 1 <= day <= 31:
                    raise HTTPException(
                        status_code=status.HTTP_400_BAD_REQUEST,
                        detail="Days of month must be between 1 and 31"
                    )

        # Create the recurring task pattern
        db_recurring_pattern = RecurringTaskPattern.model_validate(recurring_pattern)
        session.add(db_recurring_pattern)
        session.commit()
        session.refresh(db_recurring_pattern)

        # Generate the first occurrence if start date is today or in the past
        if recurring_pattern.start_date <= date.today():
            create_task_from_recurring_pattern(session, db_recurring_pattern)

        return db_recurring_pattern

    except HTTPException:
        raise
    except Exception as e:
        session.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating recurring task pattern: {str(e)}"
        )


@router.get("/users/{user_id}/recurring-tasks", response_model=List[RecurringTaskPatternRead])
def get_recurring_task_patterns(
    user_id: UUID,
    skip: int = 0,
    limit: int = 100,
    session: Session = Depends(get_session)
):
    """
    Get all recurring task patterns for a specific user.
    """
    try:
        statement = select(RecurringTaskPattern).where(
            RecurringTaskPattern.user_id == user_id
        ).offset(skip).limit(limit)

        recurring_patterns = session.exec(statement).all()
        return recurring_patterns

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving recurring task patterns: {str(e)}"
        )


@router.get("/recurring-tasks/{pattern_id}", response_model=RecurringTaskPatternRead)
def get_recurring_task_pattern(
    pattern_id: UUID,
    session: Session = Depends(get_session)
):
    """
    Get a specific recurring task pattern by ID.
    """
    try:
        recurring_pattern = session.get(RecurringTaskPattern, pattern_id)
        if not recurring_pattern:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Recurring task pattern not found"
            )

        return recurring_pattern

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving recurring task pattern: {str(e)}"
        )


@router.put("/recurring-tasks/{pattern_id}", response_model=RecurringTaskPatternRead)
def update_recurring_task_pattern(
    pattern_id: UUID,
    recurring_pattern_update: RecurringTaskPatternUpdate,
    session: Session = Depends(get_session)
):
    """
    Update a recurring task pattern.
    """
    try:
        db_recurring_pattern = session.get(RecurringTaskPattern, pattern_id)
        if not db_recurring_pattern:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Recurring task pattern not found"
            )

        # Update the pattern with the provided values
        update_data = recurring_pattern_update.model_dump(exclude_unset=True)
        for field, value in update_data.items():
            setattr(db_recurring_pattern, field, value)

        session.add(db_recurring_pattern)
        session.commit()
        session.refresh(db_recurring_pattern)

        return db_recurring_pattern

    except HTTPException:
        raise
    except Exception as e:
        session.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating recurring task pattern: {str(e)}"
        )


@router.delete("/recurring-tasks/{pattern_id}")
def delete_recurring_task_pattern(
    pattern_id: UUID,
    session: Session = Depends(get_session)
):
    """
    Delete a recurring task pattern.
    This does NOT delete the tasks that were generated from this pattern.
    """
    try:
        db_recurring_pattern = session.get(RecurringTaskPattern, pattern_id)
        if not db_recurring_pattern:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Recurring task pattern not found"
            )

        session.delete(db_recurring_pattern)
        session.commit()

        return {"message": "Recurring task pattern deleted successfully"}

    except HTTPException:
        raise
    except Exception as e:
        session.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error deleting recurring task pattern: {str(e)}"
        )


@router.post("/recurring-tasks/{pattern_id}/generate-occurrence")
def generate_next_occurrence(
    pattern_id: UUID,
    session: Session = Depends(get_session)
):
    """
    Manually generate the next occurrence for a recurring task pattern.
    This is useful for testing or when the automated system needs to be triggered.
    """
    try:
        recurring_pattern = session.get(RecurringTaskPattern, pattern_id)
        if not recurring_pattern:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Recurring task pattern not found"
            )

        # Create the next occurrence based on the pattern
        created_task = create_task_from_recurring_pattern(session, recurring_pattern)

        return {
            "message": "Next occurrence generated successfully",
            "task_id": created_task.id,
            "task_title": created_task.title
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error generating next occurrence: {str(e)}"
        )