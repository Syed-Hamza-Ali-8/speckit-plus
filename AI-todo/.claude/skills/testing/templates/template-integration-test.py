import pytest
from backend.workflows.task_workflow import advance_task
from backend.models.task import Task
from backend.db.session import SessionLocal

@pytest.fixture
def db_session():
    session = SessionLocal()
    yield session
    session.close()

def test_workflow_completion(db_session):
    task = Task(title="Integration Task", assigned_to="user_123")
    db_session.add(task)
    db_session.commit()
    advance_task(task.id, "completed", db_session)
    assert task.status == "completed"
