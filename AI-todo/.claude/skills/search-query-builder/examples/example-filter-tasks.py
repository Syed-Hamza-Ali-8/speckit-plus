from templates.template-query-builder import build_query
from templates.template-pagination import paginate
from backend.models.task import Task
from backend.db.session import SessionLocal

session = SessionLocal()

filters = {
    "status": "pending",
    "assigned_to": "user_123",
    "due_date": {"range": ["2025-12-01", "2025-12-31"]}
}

query = build_query(Task, filters, sort=["-priority"])
result = paginate(session.exec(query), limit=5, offset=0)
print(result)
