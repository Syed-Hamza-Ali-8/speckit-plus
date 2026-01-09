# Example Workflow Integration
- When workflow task moves to "completed":
    → log_event(actor_id="system", action_type="task.completed", resource_type="task", resource_id=task.id)
- When async task finishes:
    → log_event(actor_id="system", action_type="async_task.completed", resource_type="job", resource_id=job.id)
