#!/usr/bin/env python3
"""
Notification Service for Phase V Todo App
Consumes reminder events from Kafka and sends notifications to users
"""

import asyncio
import json
import logging
import os
from datetime import datetime
from typing import Dict, Any

from aiokafka import AIOKafkaConsumer
from sqlmodel import create_engine, Session
from sqlalchemy import text

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Database configuration
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://postgres:postgres@localhost:5432/todo")
engine = create_engine(DATABASE_URL, echo=True)


class NotificationService:
    def __init__(self):
        self.consumer = None
        self.running = False
        self.bootstrap_servers = os.getenv("KAFKA_BOOTSTRAP_SERVERS", "localhost:9092")

    async def start(self):
        """Start the notification service consumer."""
        logger.info("Starting Notification Service...")

        # Create Kafka consumer
        self.consumer = AIOKafkaConsumer(
            'reminders',
            bootstrap_servers=self.bootstrap_servers,
            value_deserializer=lambda x: json.loads(x.decode('utf-8')),
            group_id='notification-service-group'
        )

        await self.consumer.start()
        logger.info("Kafka consumer started for reminders topic")

        self.running = True

        try:
            async for msg in self.consumer:
                await self.handle_reminder_event(msg.value)
        except Exception as e:
            logger.error(f"Error in notification service: {e}")
        finally:
            await self.stop()

    async def handle_reminder_event(self, event_data: Dict[str, Any]):
        """Handle reminder events from Kafka."""
        logger.info(f"Received reminder event: {event_data}")

        try:
            task_id = event_data.get("task_id")
            user_id = event_data.get("user_id")
            title = event_data.get("title")
            due_at = event_data.get("due_at")
            remind_at = event_data.get("remind_at")

            # In a real implementation, this would send actual notifications
            # (email, push notification, SMS, etc.)
            logger.info(f"Sending notification for task {task_id} to user {user_id}: {title}")

            # Update reminder status in database
            with Session(engine) as session:
                # Update the reminder as sent
                update_query = text("""
                    UPDATE reminder
                    SET is_sent = TRUE, sent_at = :sent_at
                    WHERE task_id = :task_id AND user_id = :user_id
                """)
                session.execute(update_query, {
                    "sent_at": datetime.utcnow(),
                    "task_id": task_id,
                    "user_id": user_id
                })
                session.commit()

            logger.info(f"Reminder notification processed for task {task_id}")

        except Exception as e:
            logger.error(f"Failed to process reminder event: {e}")

    async def stop(self):
        """Stop the notification service."""
        logger.info("Stopping Notification Service...")
        self.running = False
        if self.consumer:
            await self.consumer.stop()


async def main():
    """Main entry point for the notification service."""
    service = NotificationService()

    try:
        await service.start()
    except KeyboardInterrupt:
        logger.info("Shutting down Notification Service...")
    finally:
        await service.stop()


if __name__ == "__main__":
    asyncio.run(main())