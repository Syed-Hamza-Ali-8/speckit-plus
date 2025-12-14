import pytest
from backend.models.task import Task

def test_create_task():
    task = Task(title="Test Task", assigned_to="user_123")
    assert task.title == "Test Task"
    assert task.assigned_to == "user_123"

def test_task_empty_title():
    with pytest.raises(ValueError):
        Task(title="", assigned_to="user_123")
