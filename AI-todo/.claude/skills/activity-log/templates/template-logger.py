from datetime import datetime

class ActivityLogger:
    def __init__(self, session=None):
        self.session = session  # optional DB session

    def log_event(self, actor_id: str, action_type: str, resource_type: str, resource_id: str, metadata: dict = None, timestamp: datetime = None):
        timestamp = timestamp or datetime.utcnow()
        log_entry = {
            "actor_id": actor_id,
            "action_type": action_type,
            "resource_type": resource_type,
            "resource_id": resource_id,
            "metadata": metadata or {},
            "timestamp": timestamp
        }

        # Persist log if session provided
        if self.session:
            from backend.models.activity_log import ActivityLog
            log_obj = ActivityLog(**log_entry)
            self.session.add(log_obj)
            self.session.commit()
        else:
            print("[ActivityLog]", log_entry)

logger = ActivityLogger()
