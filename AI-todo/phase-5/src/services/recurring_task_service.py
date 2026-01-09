#!/usr/bin/env python3
"""
Recurring Task Service for Phase V Todo App
Consumes recurring task events and generates new task instances
"""

import asyncio
import json
import logging
import os
from datetime import datetime, timedelta
from typing import Dict, Any

from aiokafka import AIOKafkaConsumer
from sqlmodel import create_engine, Session, select
from sqlalchemy import text

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Database configuration
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://postgres:postgres@localhost:5432/todo")
engine = create_engine(DATABASE_URL, echo=True)


class RecurringTaskService:
    def __init__(self):
        self.consumer = None
        self.running = False
        self.bootstrap_servers = os.getenv("KAFKA_BOOTSTRAP_SERVERS", "localhost:9092")

    async def start(self):
        """Start the recurring task service consumer."""
        logger.info("Starting Recurring Task Service...")

        # Create Kafka consumer
        self.consumer = AIOKafkaConsumer(
            'task-events',
            bootstrap_servers=self.bootstrap_servers,
            value_deserializer=lambda x: json.loads(x.decode('utf-8')),
            group_id='recurring-task-service-group'
        )

        await self.consumer.start()
        logger.info("Kafka consumer started for task-events topic")

        self.running = True

        try:
            async for msg in self.consumer:
                if msg.key and b'recurring' in msg.key or 'recurring_task' in str(msg.value):
                    await self.handle_recurring_task_event(msg.value)
        except Exception as e:
            logger.error(f"Error in recurring task service: {e}")
        finally:
            await self.stop()

    async def handle_recurring_task_event(self, event_data: Dict[str, Any]):
        """Handle recurring task events from Kafka."""
        logger.info(f"Received recurring task event: {event_data}")

        try:
            event_type = event_data.get("event_type", "")

            if event_type == "recurring_task_generated":
                pattern_id = event_data.get("pattern_id")
                user_id = event_data.get("user_id")
                next_task_data = event_data.get("next_task_data")

                logger.info(f"Processing recurring task generation for pattern {pattern_id}")

                # In a real implementation, this would create new task instances based on the pattern
                # For now, we'll just log the action
                logger.info(f"Generated new task instance for pattern {pattern_id} for user {user_id}")

            elif event_type == "task_completed":
                # Check if this task has a recurring pattern and schedule the next occurrence
                task_id = event_data.get("task_id")
                user_id = event_data.get("user_id")
                task_data = event_data.get("task_data", {})

                # Check if the completed task has a recurring pattern
                with Session(engine) as session:
                    recurring_pattern_id = task_data.get("recurring_pattern_id")
                    if recurring_pattern_id:
                        # This is a recurring task that was completed - generate the next occurrence
                        logger.info(f"Scheduling next occurrence for recurring task pattern {recurring_pattern_id}")

                        # Generate next task based on pattern
                        await self.generate_next_task(session, recurring_pattern_id, user_id)

        except Exception as e:
            logger.error(f"Failed to process recurring task event: {e}")

    async def generate_next_task(self, session: Session, pattern_id: int, user_id: str):
        """Generate the next task instance based on the recurring pattern."""
        try:
            # Query the recurring pattern from the database
            pattern_query = text("""
                SELECT base_task_title, base_task_description, pattern_type, interval,
                       start_date, end_date, weekdays, days_of_month
                FROM recurringtaskpattern
                WHERE id = :pattern_id
            """)
            pattern_result = session.execute(pattern_query, {"pattern_id": pattern_id}).fetchone()

            if not pattern_result:
                logger.warning(f"Recurring pattern {pattern_id} not found")
                return

            # Extract pattern details
            (base_task_title, base_task_description, pattern_type, interval,
             start_date, end_date, weekdays, days_of_month) = pattern_result

            # Calculate next occurrence based on pattern
            next_occurrence = self.calculate_next_occurrence(
                pattern_type, interval, start_date, weekdays, days_of_month
            )

            if end_date and next_occurrence.date() > end_date:
                logger.info(f"Recurring pattern {pattern_id} has reached end date")
                return

            # Create new task based on pattern
            create_task_query = text("""
                INSERT INTO task (title, description, user_id, status, priority, tags,
                                 due_date, is_reminder_sent, is_recurring,
                                 recurring_pattern_id, created_at, updated_at, created_by, updated_by)
                VALUES (:title, :description, :user_id, :status, :priority, :tags,
                        :due_date, :is_reminder_sent, :is_recurring,
                        :recurring_pattern_id, :created_at, :updated_at, :created_by, :updated_by)
                RETURNING id
            """)

            new_task_data = {
                "title": f"{base_task_title} (Recurring)",
                "description": base_task_description,
                "user_id": user_id,
                "status": "pending",
                "priority": "medium",
                "tags": [],
                "due_date": next_occurrence.date(),
                "is_reminder_sent": False,
                "is_recurring": True,
                "recurring_pattern_id": pattern_id,
                "created_at": datetime.utcnow(),
                "updated_at": datetime.utcnow(),
                "created_by": user_id,
                "updated_by": user_id
            }

            result = session.execute(create_task_query, new_task_data)
            new_task_id = result.fetchone()[0]
            session.commit()

            logger.info(f"Created new recurring task {new_task_id} for pattern {pattern_id}")

            # Publish event for the new task
            from services.kafka_producer import publish_task_created_event
            await publish_task_created_event(new_task_data, user_id)

        except Exception as e:
            logger.error(f"Failed to generate next task for pattern {pattern_id}: {e}")

    def calculate_next_occurrence(self, pattern_type: str, interval: int, start_date: datetime,
                                  weekdays: list, days_of_month: list):
        """Calculate the next occurrence based on the pattern type."""
        now = datetime.utcnow()

        if pattern_type == "daily":
            return now + timedelta(days=interval)
        elif pattern_type == "weekly":
            return now + timedelta(weeks=interval)
        elif pattern_type == "monthly":
            # Simple monthly calculation (same day of month)
            next_month = now.month + interval
            next_year = now.year
            if next_month > 12:
                next_year += next_month // 12
                next_month = next_month % 12
            if next_month == 0:
                next_month = 12
                next_year -= 1

            # Handle days that don't exist in shorter months (e.g., Feb 30th)
            try:
                next_date = now.replace(year=next_year, month=next_month)
            except ValueError:
                # Day doesn't exist in that month, use last day of month
                if next_month in [4, 6, 9, 11]:
                    next_date = now.replace(year=next_year, month=next_month, day=30)
                elif next_month == 2:
                    if (next_year % 4 == 0 and next_year % 100 != 0) or (next_year % 400 == 0):
                        next_date = now.replace(year=next_year, month=next_month, day=29)
                    else:
                        next_date = now.replace(year=next_year, month=next_month, day=28)
                else:
                    next_date = now.replace(year=next_year, month=next_month, day=31)

            return next_date
        elif pattern_type == "yearly":
            return now.replace(year=now.year + interval)
        else:
            return now + timedelta(days=interval)  # Default to daily

    async def stop(self):
        """Stop the recurring task service."""
        logger.info("Stopping Recurring Task Service...")
        self.running = False
        if self.consumer:
            await self.consumer.stop()


async def main():
    """Main entry point for the recurring task service."""
    service = RecurringTaskService()

    try:
        await service.start()
    except KeyboardInterrupt:
        logger.info("Shutting down Recurring Task Service...")
    finally:
        await service.stop()


if __name__ == "__main__":
    asyncio.run(main())