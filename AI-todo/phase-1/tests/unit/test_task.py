"""
Unit tests for Task model
"""
import pytest
from src.models.task import Task


def test_task_creation():
    """Test creating a task with valid data"""
    task = Task(id=1, title="Test Task", description="Test Description", completed=False)
    assert task.id == 1
    assert task.title == "Test Task"
    assert task.description == "Test Description"
    assert task.completed is False


def test_task_creation_defaults():
    """Test creating a task with default values"""
    task = Task(id=1, title="Test Task")
    assert task.id == 1
    assert task.title == "Test Task"
    assert task.description is None
    assert task.completed is False


def test_task_title_validation():
    """Test that task title cannot be empty"""
    with pytest.raises(ValueError, match="Task title cannot be empty or contain only whitespace"):
        Task(id=1, title="", description="Test Description")


def test_task_title_whitespace_validation():
    """Test that task title cannot contain only whitespace"""
    with pytest.raises(ValueError, match="Task title cannot be empty or contain only whitespace"):
        Task(id=1, title="   ", description="Test Description")


def test_task_title_length_validation():
    """Test that task title cannot exceed 200 characters"""
    long_title = "x" * 201
    with pytest.raises(ValueError, match="Task title must not exceed 200 characters"):
        Task(id=1, title=long_title, description="Test Description")


def test_task_description_length_validation():
    """Test that task description cannot exceed 1000 characters"""
    long_description = "x" * 1001
    with pytest.raises(ValueError, match="Task description must not exceed 1000 characters"):
        Task(id=1, title="Test Task", description=long_description)