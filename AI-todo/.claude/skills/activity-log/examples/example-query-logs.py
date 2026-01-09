from templates.template-audit-query import query_logs
from backend.db.session import SessionLocal

session = SessionLocal()

results = query_logs(session, actor_id="user_123", action_type="task.created")
for log in results:
    print(log)
