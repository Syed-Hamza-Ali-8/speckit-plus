"""
TaskService: Business logic for task operations
"""
import sys
import os
from typing import List, Optional

# Add the project root to the Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, project_root)

from src.models.task import Task, TaskManager


class TaskService:
    """
    Business logic layer for task operations
    """
    def __init__(self, storage_file="tasks.json"):
        self.task_manager = TaskManager(storage_file)

    def add_task(self, title: str, description: Optional[str] = None) -> Task:
        """Add a new task with validation"""
        # Validate inputs
        if not title or not title.strip():
            raise ValueError("Task title cannot be empty or contain only whitespace")
        if len(title) > 200:
            raise ValueError("Task title must not exceed 200 characters")
        if description and len(description) > 1000:
            raise ValueError("Task description must not exceed 1000 characters")

        return self.task_manager.add_task(title, description)

    def list_tasks(self) -> List[Task]:
        """Get all tasks"""
        return self.task_manager.get_all_tasks()

    def get_task(self, task_id: int) -> Optional[Task]:
        """Get a specific task by ID"""
        return self.task_manager.get_task_by_id(task_id)

    def update_task(self, task_id: int, title: Optional[str] = None,
                   description: Optional[str] = None) -> Optional[Task]:
        """Update task details with validation"""
        # Validate inputs if provided
        if title is not None:
            if not title or not title.strip():
                raise ValueError("Task title cannot be empty or contain only whitespace")
            if len(title) > 200:
                raise ValueError("Task title must not exceed 200 characters")
        if description is not None:
            if description and len(description) > 1000:
                raise ValueError("Task description must not exceed 1000 characters")

        return self.task_manager.update_task(task_id, title, description)

    def delete_task(self, task_id: int) -> bool:
        """Delete a task by ID"""
        return self.task_manager.delete_task(task_id)

    def mark_complete(self, task_id: int) -> bool:
        """Mark a task as complete"""
        task = self.task_manager.get_task_by_id(task_id)
        if not task:
            return False
        return self.task_manager.mark_complete(task_id)

    def mark_incomplete(self, task_id: int) -> bool:
        """Mark a task as incomplete"""
        task = self.task_manager.get_task_by_id(task_id)
        if not task:
            return False
        return self.task_manager.mark_incomplete(task_id)