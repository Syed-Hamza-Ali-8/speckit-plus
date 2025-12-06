"""
Unit tests for TaskService
"""
import pytest
from src.services.task_service import TaskService


def test_add_task():
    """Test adding a task"""
    service = TaskService()
    task = service.add_task("Test Task", "Test Description")

    assert task.id == 1
    assert task.title == "Test Task"
    assert task.description == "Test Description"
    assert task.completed is False


def test_add_task_without_description():
    """Test adding a task without description"""
    service = TaskService()
    task = service.add_task("Test Task")

    assert task.id == 1
    assert task.title == "Test Task"
    assert task.description is None
    assert task.completed is False


def test_add_task_title_validation():
    """Test adding a task with empty title raises error"""
    service = TaskService()
    with pytest.raises(ValueError, match="Task title cannot be empty or contain only whitespace"):
        service.add_task("", "Test Description")


def test_add_task_title_whitespace_validation():
    """Test adding a task with whitespace-only title raises error"""
    service = TaskService()
    with pytest.raises(ValueError, match="Task title cannot be empty or contain only whitespace"):
        service.add_task("   ", "Test Description")


def test_add_task_title_length_validation():
    """Test adding a task with too long title raises error"""
    service = TaskService()
    long_title = "x" * 201
    with pytest.raises(ValueError, match="Task title must not exceed 200 characters"):
        service.add_task(long_title, "Test Description")


def test_list_tasks():
    """Test listing tasks"""
    service = TaskService()
    service.add_task("Task 1", "Description 1")
    service.add_task("Task 2", "Description 2")

    tasks = service.list_tasks()
    assert len(tasks) == 2
    assert tasks[0].title == "Task 1"
    assert tasks[1].title == "Task 2"


def test_get_task():
    """Test getting a specific task"""
    service = TaskService()
    created_task = service.add_task("Test Task", "Test Description")

    retrieved_task = service.get_task(created_task.id)
    assert retrieved_task is not None
    assert retrieved_task.id == created_task.id
    assert retrieved_task.title == "Test Task"


def test_get_nonexistent_task():
    """Test getting a non-existent task returns None"""
    service = TaskService()
    task = service.get_task(999)
    assert task is None


def test_update_task():
    """Test updating a task"""
    service = TaskService()
    original_task = service.add_task("Original Title", "Original Description")

    updated_task = service.update_task(original_task.id, "New Title", "New Description")

    assert updated_task is not None
    assert updated_task.title == "New Title"
    assert updated_task.description == "New Description"


def test_update_task_partial():
    """Test updating only title or description"""
    service = TaskService()
    original_task = service.add_task("Original Title", "Original Description")

    # Update only title
    updated_task = service.update_task(original_task.id, title="New Title")
    assert updated_task.title == "New Title"
    assert updated_task.description == "Original Description"

    # Update only description
    updated_task = service.update_task(original_task.id, description="New Description")
    assert updated_task.title == "New Title"
    assert updated_task.description == "New Description"


def test_delete_task():
    """Test deleting a task"""
    service = TaskService()
    task = service.add_task("Test Task", "Test Description")

    result = service.delete_task(task.id)
    assert result is True

    # Verify task is gone
    remaining_tasks = service.list_tasks()
    assert len(remaining_tasks) == 0


def test_delete_nonexistent_task():
    """Test deleting a non-existent task returns False"""
    service = TaskService()
    result = service.delete_task(999)
    assert result is False


def test_mark_complete():
    """Test marking a task as complete"""
    service = TaskService()
    task = service.add_task("Test Task", "Test Description")

    result = service.mark_complete(task.id)
    assert result is True

    # Verify task is marked complete
    updated_task = service.get_task(task.id)
    assert updated_task is not None
    assert updated_task.completed is True


def test_mark_incomplete():
    """Test marking a task as incomplete"""
    service = TaskService()
    task = service.add_task("Test Task", "Test Description")

    # First mark as complete
    service.mark_complete(task.id)

    # Then mark as incomplete
    result = service.mark_incomplete(task.id)
    assert result is True

    # Verify task is marked incomplete
    updated_task = service.get_task(task.id)
    assert updated_task is not None
    assert updated_task.completed is False