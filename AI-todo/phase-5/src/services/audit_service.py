#!/usr/bin/env python3
"""
Audit Service for Phase V Todo App
Consumes task events and creates audit trails
"""

import asyncio
import json
import logging
import os
from datetime import datetime
from typing import Dict, Any

from aiokafka import AIOKafkaConsumer
from sqlmodel import create_engine, Session, SQLModel
from sqlalchemy import text

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Database configuration
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://postgres:postgres@localhost:5432/todo")
engine = create_engine(DATABASE_URL, echo=True)


class AuditService:
    def __init__(self):
        self.consumer = None
        self.running = False
        self.bootstrap_servers = os.getenv("KAFKA_BOOTSTRAP_SERVERS", "localhost:9092")

    async def start(self):
        """Start the audit service consumer."""
        logger.info("Starting Audit Service...")

        # Create Kafka consumer
        self.consumer = AIOKafkaConsumer(
            'task-events',
            bootstrap_servers=self.bootstrap_servers,
            value_deserializer=lambda x: json.loads(x.decode('utf-8')),
            group_id='audit-service-group'
        )

        await self.consumer.start()
        logger.info("Kafka consumer started for task-events topic")

        self.running = True

        try:
            async for msg in self.consumer:
                await self.handle_task_event(msg.value)
        except Exception as e:
            logger.error(f"Error in audit service: {e}")
        finally:
            await self.stop()

    async def handle_task_event(self, event_data: Dict[str, Any]):
        """Handle task events and create audit entries."""
        logger.info(f"Received task event: {event_data}")

        try:
            event_type = event_data.get("event_type", "unknown")
            task_id = event_data.get("task_id")
            user_id = event_data.get("user_id")
            timestamp = event_data.get("timestamp", datetime.utcnow().isoformat())
            task_data = event_data.get("task_data", {})
            previous_task_data = event_data.get("previous_task_data", {})

            # Create audit entry in database
            with Session(engine) as session:
                audit_entry_query = text("""
                    INSERT INTO taskhistory (task_id, user_id, action, previous_state, new_state, created_at)
                    VALUES (:task_id, :user_id, :action, :previous_state, :new_state, :timestamp)
                """)

                session.execute(audit_entry_query, {
                    "task_id": task_id,
                    "user_id": user_id,
                    "action": event_type,
                    "previous_state": json.dumps(previous_task_data) if previous_task_data else None,
                    "new_state": json.dumps(task_data),
                    "timestamp": datetime.fromisoformat(timestamp) if isinstance(timestamp, str) else timestamp
                })

                session.commit()
                logger.info(f"Audit entry created for task {task_id}, action: {event_type}")

        except Exception as e:
            logger.error(f"Failed to process audit event: {e}")

    async def stop(self):
        """Stop the audit service."""
        logger.info("Stopping Audit Service...")
        self.running = False
        if self.consumer:
            await self.consumer.stop()


async def main():
    """Main entry point for the audit service."""
    service = AuditService()

    try:
        await service.start()
    except KeyboardInterrupt:
        logger.info("Shutting down Audit Service...")
    finally:
        await service.stop()


if __name__ == "__main__":
    asyncio.run(main())