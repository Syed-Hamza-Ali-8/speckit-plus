"""
Integration tests for CLI commands
"""
import io
import sys
from contextlib import redirect_stdout
import pytest
from src.cli.cli import CLI


def test_add_task_cli():
    """Test adding a task via CLI"""
    cli = CLI()
    # Capture stdout to verify output
    captured_output = io.StringIO()

    with redirect_stdout(captured_output):
        cli.add_task("Test Task", "Test Description")

    output = captured_output.getvalue().strip()
    assert "Task added successfully" in output


def test_add_task_cli_without_description():
    """Test adding a task without description via CLI"""
    cli = CLI()
    captured_output = io.StringIO()

    with redirect_stdout(captured_output):
        cli.add_task("Test Task")

    output = captured_output.getvalue().strip()
    assert "Task added successfully" in output


def test_list_tasks_cli():
    """Test listing tasks via CLI"""
    cli = CLI()
    cli.service.add_task("Task 1", "Description 1")
    cli.service.add_task("Task 2", "Description 2")

    captured_output = io.StringIO()

    with redirect_stdout(captured_output):
        cli.list_tasks()

    output = captured_output.getvalue()
    assert "Task 1" in output
    assert "Task 2" in output
    assert "No tasks found." not in output


def test_list_empty_tasks_cli():
    """Test listing when no tasks exist"""
    cli = CLI()
    captured_output = io.StringIO()

    with redirect_stdout(captured_output):
        cli.list_tasks()

    output = captured_output.getvalue().strip()
    assert "No tasks found." in output


def test_update_task_cli():
    """Test updating a task via CLI"""
    cli = CLI()
    original_task = cli.service.add_task("Original Title", "Original Description")

    captured_output = io.StringIO()

    with redirect_stdout(captured_output):
        cli.update_task(original_task.id, "New Title", "New Description")

    output = captured_output.getvalue().strip()
    assert f"Task {original_task.id} updated successfully" in output


def test_update_nonexistent_task_cli():
    """Test updating a non-existent task via CLI"""
    cli = CLI()
    captured_output = io.StringIO()

    with redirect_stdout(captured_output):
        cli.update_task(999, "New Title", "New Description")

    output = captured_output.getvalue().strip()
    assert "Task with ID 999 not found" in output


def test_delete_task_cli():
    """Test deleting a task via CLI"""
    cli = CLI()
    task = cli.service.add_task("Test Task", "Test Description")

    captured_output = io.StringIO()

    with redirect_stdout(captured_output):
        cli.delete_task(task.id)

    output = captured_output.getvalue().strip()
    assert f"Task {task.id} deleted successfully" in output


def test_delete_nonexistent_task_cli():
    """Test deleting a non-existent task via CLI"""
    cli = CLI()
    captured_output = io.StringIO()

    with redirect_stdout(captured_output):
        cli.delete_task(999)

    output = captured_output.getvalue().strip()
    assert "Task with ID 999 not found" in output


def test_mark_complete_cli():
    """Test marking a task as complete via CLI"""
    cli = CLI()
    task = cli.service.add_task("Test Task", "Test Description")

    captured_output = io.StringIO()

    with redirect_stdout(captured_output):
        cli.mark_complete(task.id)

    output = captured_output.getvalue().strip()
    assert f"Task {task.id} marked as complete" in output


def test_mark_incomplete_cli():
    """Test marking a task as incomplete via CLI"""
    cli = CLI()
    task = cli.service.add_task("Test Task", "Test Description")

    # First mark as complete
    cli.service.mark_complete(task.id)

    captured_output = io.StringIO()

    with redirect_stdout(captured_output):
        cli.mark_incomplete(task.id)

    output = captured_output.getvalue().strip()
    assert f"Task {task.id} marked as incomplete" in output


def test_error_handling_empty_title():
    """Test CLI error handling for empty title"""
    cli = CLI()
    captured_output = io.StringIO()

    with redirect_stdout(captured_output):
        cli.add_task("")  # Empty title should cause error

    output = captured_output.getvalue().strip()
    assert "Error:" in output
    assert "Task title cannot be empty" in output