import httpx
import json
import logging
from typing import Dict, Any, Optional, List
from datetime import datetime
import os


# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class DaprService:
    def __init__(self):
        self.dapr_http_endpoint = os.getenv("DAPR_HTTP_ENDPOINT", "http://localhost:3500")
        self.dapr_grpc_endpoint = os.getenv("DAPR_GRPC_ENDPOINT", "http://localhost:50001")
        self.timeout = httpx.Timeout(30.0, connect=5.0)

    async def publish_event(self, pubsub_name: str, topic_name: str, data: Dict[str, Any]):
        """
        Publish an event to a Dapr pubsub component.
        """
        try:
            url = f"{self.dapr_http_endpoint}/v1.0/publish/{pubsub_name}/{topic_name}"

            async with httpx.AsyncClient(timeout=self.timeout) as client:
                response = await client.post(url, json=data)

                if response.status_code == 200:
                    logger.info(f"Event published to {pubsub_name}/{topic_name}")
                    return True
                else:
                    logger.error(f"Failed to publish event: {response.status_code} - {response.text}")
                    return False
        except Exception as e:
            logger.error(f"Exception while publishing event: {e}")
            return False

    async def get_state(self, store_name: str, key: str) -> Optional[Dict[str, Any]]:
        """
        Get state from a Dapr state store.
        """
        try:
            url = f"{self.dapr_http_endpoint}/v1.0/state/{store_name}/{key}"

            async with httpx.AsyncClient(timeout=self.timeout) as client:
                response = await client.get(url)

                if response.status_code == 200:
                    return response.json()
                elif response.status_code == 404:
                    logger.info(f"State not found for key: {key}")
                    return None
                else:
                    logger.error(f"Failed to get state: {response.status_code} - {response.text}")
                    return None
        except Exception as e:
            logger.error(f"Exception while getting state: {e}")
            return None

    async def save_state(self, store_name: str, key: str, value: Dict[str, Any], etag: Optional[str] = None):
        """
        Save state to a Dapr state store.
        """
        try:
            url = f"{self.dapr_http_endpoint}/v1.0/state/{store_name}"

            state_entry = {
                "key": key,
                "value": value
            }

            if etag:
                state_entry["etag"] = etag
                state_entry["options"] = {
                    "concurrency": "first-write",
                    "consistency": "strong"
                }

            async with httpx.AsyncClient(timeout=self.timeout) as client:
                response = await client.post(url, json=[state_entry])

                if response.status_code == 200:
                    logger.info(f"State saved for key: {key}")
                    return True
                else:
                    logger.error(f"Failed to save state: {response.status_code} - {response.text}")
                    return False
        except Exception as e:
            logger.error(f"Exception while saving state: {e}")
            return False

    async def delete_state(self, store_name: str, key: str):
        """
        Delete state from a Dapr state store.
        """
        try:
            url = f"{self.dapr_http_endpoint}/v1.0/state/{store_name}/{key}"

            async with httpx.AsyncClient(timeout=self.timeout) as client:
                response = await client.delete(url)

                if response.status_code == 200:
                    logger.info(f"State deleted for key: {key}")
                    return True
                else:
                    logger.error(f"Failed to delete state: {response.status_code} - {response.text}")
                    return False
        except Exception as e:
            logger.error(f"Exception while deleting state: {e}")
            return False

    async def invoke_service(self, app_id: str, method: str, data: Optional[Dict[str, Any]] = None):
        """
        Invoke a method on another service via Dapr service invocation.
        """
        try:
            url = f"{self.dapr_http_endpoint}/v1.0/invoke/{app_id}/method/{method}"

            async with httpx.AsyncClient(timeout=self.timeout) as client:
                if data:
                    response = await client.post(url, json=data)
                else:
                    response = await client.get(url)

                if response.status_code in [200, 201, 204]:
                    try:
                        return response.json()
                    except json.JSONDecodeError:
                        # If response is not JSON, return text
                        return response.text
                else:
                    logger.error(f"Failed to invoke service {app_id}/{method}: {response.status_code} - {response.text}")
                    return None
        except Exception as e:
            logger.error(f"Exception while invoking service {app_id}/{method}: {e}")
            return None

    async def get_secret(self, secret_store_name: str, key: str) -> Optional[str]:
        """
        Get a secret from a Dapr secret store.
        """
        try:
            url = f"{self.dapr_http_endpoint}/v1.0/secrets/{secret_store_name}/{key}"

            async with httpx.AsyncClient(timeout=self.timeout) as client:
                response = await client.get(url)

                if response.status_code == 200:
                    secrets = response.json()
                    return secrets.get(key)
                else:
                    logger.error(f"Failed to get secret: {response.status_code} - {response.text}")
                    return None
        except Exception as e:
            logger.error(f"Exception while getting secret: {e}")
            return None

    async def get_bulk_secret(self, secret_store_name: str) -> Optional[Dict[str, str]]:
        """
        Get all secrets from a Dapr secret store.
        """
        try:
            url = f"{self.dapr_http_endpoint}/v1.0/secrets/{secret_store_name}/bulk"

            async with httpx.AsyncClient(timeout=self.timeout) as client:
                response = await client.get(url)

                if response.status_code == 200:
                    return response.json()
                else:
                    logger.error(f"Failed to get bulk secrets: {response.status_code} - {response.text}")
                    return None
        except Exception as e:
            logger.error(f"Exception while getting bulk secrets: {e}")
            return None

    # Convenience methods for specific use cases in the Todo app

    async def get_conversation_state(self, conversation_id: str) -> Optional[Dict[str, Any]]:
        """Get conversation state from Dapr state store."""
        return await self.get_state("statestore", f"conversation-{conversation_id}")

    async def save_conversation_state(self, conversation_id: str, state: Dict[str, Any]) -> bool:
        """Save conversation state to Dapr state store."""
        return await self.save_state("statestore", f"conversation-{conversation_id}", state)

    async def get_user_preferences(self, user_id: str) -> Optional[Dict[str, Any]]:
        """Get user preferences from Dapr state store."""
        return await self.get_state("statestore", f"user-preferences-{user_id}")

    async def save_user_preferences(self, user_id: str, preferences: Dict[str, Any]) -> bool:
        """Save user preferences to Dapr state store."""
        return await self.save_state("statestore", f"user-preferences-{user_id}", preferences)

    async def publish_task_event(self, topic_name: str, event_data: Dict[str, Any]) -> bool:
        """Publish a task-related event via Dapr pubsub."""
        return await self.publish_event("kafka-pubsub", topic_name, event_data)

    async def publish_reminder_event(self, reminder_data: Dict[str, Any]) -> bool:
        """Publish a reminder event via Dapr pubsub."""
        return await self.publish_event("kafka-pubsub", "reminders", reminder_data)

    async def publish_task_update_event(self, update_data: Dict[str, Any]) -> bool:
        """Publish a task update event via Dapr pubsub."""
        return await self.publish_event("kafka-pubsub", "task-updates", update_data)

    async def call_notification_service(self, notification_data: Dict[str, Any]) -> Optional[Any]:
        """Call the notification service via Dapr service invocation."""
        return await self.invoke_service("notification-service", "send-notification", notification_data)

    async def call_recurring_task_service(self, task_data: Dict[str, Any]) -> Optional[Any]:
        """Call the recurring task service via Dapr service invocation."""
        return await self.invoke_service("recurring-task-service", "process-completion", task_data)

    async def call_audit_service(self, audit_data: Dict[str, Any]) -> Optional[Any]:
        """Call the audit service via Dapr service invocation."""
        return await self.invoke_service("audit-service", "log-event", audit_data)

    async def call_websocket_service(self, sync_data: Dict[str, Any]) -> Optional[Any]:
        """Call the WebSocket service for real-time synchronization."""
        return await self.invoke_service("websocket-service", "sync-task", sync_data)


# Global Dapr service instance
dapr_service = DaprService()


async def get_dapr_service():
    """Get the global Dapr service instance."""
    return dapr_service


# Example usage functions
async def publish_task_event_dapr(event_type: str, task_data: Dict[str, Any], user_id: str):
    """Publish a task event using Dapr pubsub."""
    dapr_svc = await get_dapr_service()

    event = {
        "event_type": event_type,
        "task_id": task_data.get("id"),
        "user_id": user_id,
        "timestamp": datetime.utcnow().isoformat(),
        "task_data": task_data
    }

    return await dapr_svc.publish_task_event("task-events", event)


async def publish_reminder_event_dapr(task_id: int, user_id: str, title: str, due_at: datetime, remind_at: datetime):
    """Publish a reminder event using Dapr pubsub."""
    dapr_svc = await get_dapr_service()

    event = {
        "task_id": task_id,
        "user_id": user_id,
        "title": title,
        "due_at": due_at.isoformat() if due_at else None,
        "remind_at": remind_at.isoformat() if remind_at else None,
        "notification_method": "push",
        "created_at": datetime.utcnow().isoformat()
    }

    return await dapr_svc.publish_reminder_event(event)


async def save_conversation_state_dapr(conversation_id: str, state: Dict[str, Any]):
    """Save conversation state using Dapr state management."""
    dapr_svc = await get_dapr_service()
    return await dapr_svc.save_conversation_state(conversation_id, state)


async def get_conversation_state_dapr(conversation_id: str):
    """Get conversation state using Dapr state management."""
    dapr_svc = await get_dapr_service()
    return await dapr_svc.get_conversation_state(conversation_id)


async def call_notification_service_dapr(notification_data: Dict[str, Any]):
    """Call notification service using Dapr service invocation."""
    dapr_svc = await get_dapr_service()
    return await dapr_svc.call_notification_service(notification_data)


async def call_recurring_task_service_dapr(task_data: Dict[str, Any]):
    """Call recurring task service using Dapr service invocation."""
    dapr_svc = await get_dapr_service()
    return await dapr_svc.call_recurring_task_service(task_data)