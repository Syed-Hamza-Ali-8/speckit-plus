from templates.template-query-builder import build_query
from backend.models.user import User
from backend.db.session import SessionLocal

session = SessionLocal()

filters = {"role": "admin"}
query = build_query(User, filters)
users = session.exec(query).all()
print(users)
