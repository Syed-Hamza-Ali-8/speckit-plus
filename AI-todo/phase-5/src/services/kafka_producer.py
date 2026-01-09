import asyncio
import json
import logging
from typing import Dict, Any
from aiokafka import AIOKafkaProducer
import os
from datetime import datetime

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class KafkaProducerService:
    def __init__(self):
        self.producer = None
        self.bootstrap_servers = os.getenv("KAFKA_BOOTSTRAP_SERVERS", "localhost:9092")
        self.sasl_username = os.getenv("KAFKA_SASL_USERNAME", "")
        self.sasl_password = os.getenv("KAFKA_SASL_PASSWORD", "")
        self.security_protocol = os.getenv("KAFKA_SECURITY_PROTOCOL", "SASL_SSL")
        self.sasl_mechanism = os.getenv("KAFKA_SASL_MECHANISM", "SCRAM-SHA-256")

    async def start(self):
        """Initialize and start the Kafka producer."""
        try:
            # Configure producer with appropriate settings
            config = {
                'bootstrap_servers': self.bootstrap_servers,
                'value_serializer': lambda v: json.dumps(v).encode('utf-8'),
                'acks': 'all',  # Ensure durability
                'max_batch_size': 16000,
                'linger_ms': 5
            }

            # Add SASL authentication if credentials are provided
            if self.sasl_username and self.sasl_password:
                config['sasl_mechanism'] = self.sasl_mechanism
                config['sasl_plain_username'] = self.sasl_username
                config['sasl_plain_password'] = self.sasl_password
                config['security_protocol'] = self.security_protocol
                logger.info(f"Connecting to Kafka with SASL authentication to {self.bootstrap_servers}")
            else:
                logger.info(f"Connecting to Kafka without authentication at {self.bootstrap_servers}")

            self.producer = AIOKafkaProducer(**config)
            await self.producer.start()
            logger.info("Kafka producer started successfully")
        except Exception as e:
            logger.error(f"Failed to start Kafka producer: {e}")
            raise

    async def stop(self):
        """Stop the Kafka producer."""
        if self.producer:
            await self.producer.stop()
            logger.info("Kafka producer stopped")

    async def send_task_event(self, event_type: str, task_data: Dict[str, Any], user_id: str, previous_task_data: Dict[str, Any] = None):
        """Send a task-related event to the task-events topic."""
        try:
            event = {
                "event_type": event_type,
                "task_id": task_data.get("id"),
                "user_id": user_id,
                "timestamp": datetime.utcnow().isoformat(),
                "task_data": task_data
            }

            if previous_task_data:
                event["previous_task_data"] = previous_task_data

            await self.producer.send_and_wait("task-events", event)
            logger.info(f"Task event sent: {event_type} for task {task_data.get('id')} by user {user_id}")

        except Exception as e:
            logger.error(f"Failed to send task event: {e}")
            raise

    async def send_reminder_event(self, task_id: int, user_id: str, title: str, due_at: datetime, remind_at: datetime):
        """Send a reminder event to the reminders topic."""
        try:
            event = {
                "task_id": task_id,
                "user_id": user_id,
                "title": title,
                "due_at": due_at.isoformat() if due_at else None,
                "remind_at": remind_at.isoformat() if remind_at else None,
                "notification_method": "push",
                "created_at": datetime.utcnow().isoformat()
            }

            await self.producer.send_and_wait("reminders", event)
            logger.info(f"Reminder event sent for task {task_id}")

        except Exception as e:
            logger.error(f"Failed to send reminder event: {e}")
            raise

    async def send_task_update_event(self, operation: str, task_data: Dict[str, Any], user_id: str):
        """Send a task update event to the task-updates topic for real-time sync."""
        try:
            event = {
                "task_id": task_data.get("id"),
                "user_id": user_id,
                "operation": operation,
                "timestamp": datetime.utcnow().isoformat(),
                "task_data": task_data
            }

            await self.producer.send_and_wait("task-updates", event)
            logger.info(f"Task update event sent: {operation} for task {task_data.get('id')}")

        except Exception as e:
            logger.error(f"Failed to send task update event: {e}")
            raise

    async def send_recurring_task_event(self, pattern_id: int, user_id: str, next_task_data: Dict[str, Any]):
        """Send an event when a recurring task is processed."""
        try:
            event = {
                "pattern_id": pattern_id,
                "user_id": user_id,
                "next_task_data": next_task_data,
                "timestamp": datetime.utcnow().isoformat(),
                "event_type": "recurring_task_generated"
            }

            await self.producer.send_and_wait("task-events", event)
            logger.info(f"Recurring task event sent for pattern {pattern_id}")

        except Exception as e:
            logger.error(f"Failed to send recurring task event: {e}")
            raise


# Global producer instance
kafka_producer_service = KafkaProducerService()


async def get_kafka_producer():
    """Get the global Kafka producer instance."""
    return kafka_producer_service


# Example usage functions
async def publish_task_created_event(task_data: Dict[str, Any], user_id: str):
    """Publish an event when a task is created."""
    producer = await get_kafka_producer()
    await producer.send_task_event("created", task_data, user_id)


async def publish_task_updated_event(task_data: Dict[str, Any], user_id: str, previous_task_data: Dict[str, Any] = None):
    """Publish an event when a task is updated."""
    producer = await get_kafka_producer()
    await producer.send_task_event("updated", task_data, user_id, previous_task_data)


async def publish_task_completed_event(task_data: Dict[str, Any], user_id: str):
    """Publish an event when a task is completed."""
    producer = await get_kafka_producer()
    await producer.send_task_event("completed", task_data, user_id)


async def publish_task_deleted_event(task_id: int, user_id: str, task_data: Dict[str, Any] = None):
    """Publish an event when a task is deleted."""
    producer = await get_kafka_producer()
    await producer.send_task_event("deleted", {"id": task_id} if not task_data else task_data, user_id)


async def publish_priority_changed_event(task_data: Dict[str, Any], user_id: str, previous_task_data: Dict[str, Any] = None):
    """Publish an event when a task's priority is changed."""
    producer = await get_kafka_producer()
    await producer.send_task_event("priority_changed", task_data, user_id, previous_task_data)


async def publish_due_date_set_event(task_data: Dict[str, Any], user_id: str, previous_task_data: Dict[str, Any] = None):
    """Publish an event when a task's due date is set."""
    producer = await get_kafka_producer()
    await producer.send_task_event("due_date_set", task_data, user_id, previous_task_data)


async def publish_tags_added_event(task_data: Dict[str, Any], user_id: str, previous_task_data: Dict[str, Any] = None):
    """Publish an event when tags are added to a task."""
    producer = await get_kafka_producer()
    await producer.send_task_event("tags_added", task_data, user_id, previous_task_data)


async def publish_reminder_scheduled_event(task_id: int, user_id: str, title: str, due_at: datetime, remind_at: datetime):
    """Publish an event when a reminder is scheduled."""
    producer = await get_kafka_producer()
    await producer.send_reminder_event(task_id, user_id, title, due_at, remind_at)


async def publish_task_update_sync_event(operation: str, task_data: Dict[str, Any], user_id: str):
    """Publish an event for real-time task synchronization."""
    producer = await get_kafka_producer()
    await producer.send_task_update_event(operation, task_data, user_id)


async def publish_recurring_task_generated_event(pattern_id: int, user_id: str, next_task_data: Dict[str, Any]):
    """Publish an event when a recurring task is generated."""
    producer = await get_kafka_producer()
    await producer.send_recurring_task_event(pattern_id, user_id, next_task_data)


# Initialize the producer when module is loaded
async def init_kafka_producer():
    """Initialize the Kafka producer service."""
    await kafka_producer_service.start()


# Cleanup function
async def close_kafka_producer():
    """Close the Kafka producer service."""
    await kafka_producer_service.stop()