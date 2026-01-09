from backend.events.event_bus import event_bus
from backend.notifications.notification_service import notification_service
from backend.workflows.MODEL_workflow import advance_MODEL
from backend.services.MODEL_service import get_MODEL

def MODEL_completed_handler(payload):
    session = payload["session"]
    model_id = payload["id"]

    # Run workflow
    advance_MODEL(model_id, "completed", session)

    # Notify user
    model = get_MODEL(model_id, session)
    notification_service.send_email(
        to=model.assigned_to_email,
        subject="Your task is complete",
        body=f"Task '{model.title}' has been completed."
    )

event_bus.on("MODEL.completed", MODEL_completed_handler)
