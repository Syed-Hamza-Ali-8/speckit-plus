"""
Contract tests for API endpoints
"""
import pytest
from src.services.task_service import TaskService


def test_add_task_contract():
    """Test that add_task follows the expected contract"""
    service = TaskService()
    task = service.add_task("Test Task", "Test Description")

    # Verify return type and properties
    assert hasattr(task, 'id')
    assert hasattr(task, 'title')
    assert hasattr(task, 'description')
    assert hasattr(task, 'completed')
    assert task.id == 1
    assert task.title == "Test Task"
    assert task.description == "Test Description"
    assert task.completed is False


def test_list_tasks_contract():
    """Test that list_tasks follows the expected contract"""
    service = TaskService()
    service.add_task("Test Task 1", "Description 1")
    service.add_task("Test Task 2", "Description 2")

    tasks = service.list_tasks()

    # Verify return type
    assert isinstance(tasks, list)
    assert len(tasks) == 2
    assert all(hasattr(task, 'id') for task in tasks)
    assert all(hasattr(task, 'title') for task in tasks)
    assert all(hasattr(task, 'description') for task in tasks)
    assert all(hasattr(task, 'completed') for task in tasks)


def test_get_task_contract():
    """Test that get_task follows the expected contract"""
    service = TaskService()
    created_task = service.add_task("Test Task", "Test Description")

    retrieved_task = service.get_task(created_task.id)

    # Verify return type and properties
    assert retrieved_task is not None
    assert hasattr(retrieved_task, 'id')
    assert hasattr(retrieved_task, 'title')
    assert hasattr(retrieved_task, 'description')
    assert hasattr(retrieved_task, 'completed')
    assert retrieved_task.id == created_task.id
    assert retrieved_task.title == "Test Task"


def test_update_task_contract():
    """Test that update_task follows the expected contract"""
    service = TaskService()
    original_task = service.add_task("Original Title", "Original Description")

    updated_task = service.update_task(original_task.id, "New Title", "New Description")

    # Verify return type and properties
    assert updated_task is not None
    assert hasattr(updated_task, 'id')
    assert hasattr(updated_task, 'title')
    assert hasattr(updated_task, 'description')
    assert hasattr(updated_task, 'completed')
    assert updated_task.id == original_task.id
    assert updated_task.title == "New Title"
    assert updated_task.description == "New Description"


def test_delete_task_contract():
    """Test that delete_task follows the expected contract"""
    service = TaskService()
    task = service.add_task("Test Task", "Test Description")

    result = service.delete_task(task.id)

    # Verify return type
    assert isinstance(result, bool)
    assert result is True

    # Verify task is deleted
    remaining_tasks = service.list_tasks()
    assert len(remaining_tasks) == 0


def test_mark_complete_contract():
    """Test that mark_complete follows the expected contract"""
    service = TaskService()
    task = service.add_task("Test Task", "Test Description")

    result = service.mark_complete(task.id)

    # Verify return type
    assert isinstance(result, bool)
    assert result is True

    # Verify task is marked complete
    updated_task = service.get_task(task.id)
    assert updated_task is not None
    assert updated_task.completed is True


def test_mark_incomplete_contract():
    """Test that mark_incomplete follows the expected contract"""
    service = TaskService()
    task = service.add_task("Test Task", "Test Description")

    # Mark as complete first
    service.mark_complete(task.id)

    # Then mark as incomplete
    result = service.mark_incomplete(task.id)

    # Verify return type
    assert isinstance(result, bool)
    assert result is True

    # Verify task is marked incomplete
    updated_task = service.get_task(task.id)
    assert updated_task is not None
    assert updated_task.completed is False