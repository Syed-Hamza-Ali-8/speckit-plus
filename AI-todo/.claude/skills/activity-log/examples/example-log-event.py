from templates.template-logger import logger

logger.log_event(
    actor_id="user_123",
    action_type="task.created",
    resource_type="task",
    resource_id="task_456",
    metadata={"title": "New Task"}
)
