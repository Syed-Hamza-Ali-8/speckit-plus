"""
Command-line interface implementation
"""
import sys
import os
from typing import Optional

# Add the project root to the Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, project_root)

from src.services.task_service import TaskService


class CLI:
    """
    Command-line interface for the todo application
    """
    def __init__(self, storage_file="tasks.json"):
        self.service = TaskService(storage_file)

        # ANSI color codes
        self.colors = {
            'red': '\033[31m',
            'green': '\033[32m',
            'yellow': '\033[33m',
            'blue': '\033[34m',
            'magenta': '\033[35m',
            'cyan': '\033[36m',
            'white': '\033[37m',
            'bold': '\033[1m',
            'reset': '\033[0m'
        }

    def colored_print(self, text, color):
        """Print text in specified color"""
        color_code = self.colors.get(color, '')
        reset_code = self.colors['reset']
        print(f"{color_code}{text}{reset_code}")

    def add_task(self, title: str, description: Optional[str] = None) -> None:
        """Add a new task"""
        try:
            task = self.service.add_task(title, description)
            self.colored_print(f"Task added successfully with ID: {task.id}", 'green')
        except ValueError as e:
            self.colored_print(f"Error: {e}", 'red')

    def list_tasks(self) -> None:
        """List all tasks"""
        tasks = self.service.list_tasks()
        if not tasks:
            self.colored_print("No tasks found.", 'yellow')
            return

        # Print header in blue
        self.colored_print("ID\tTitle\t\t\tDescription\t\tStatus", 'blue')
        print("-" * 70)
        for task in tasks:
            status = "✓" if task.completed else "○"
            description = task.description if task.description else ""
            # Truncate long titles/descriptions for display
            title_display = task.title[:20] + "..." if len(task.title) > 20 else task.title
            desc_display = description[:20] + "..." if len(description) > 20 else description

            # Color code based on completion status
            if task.completed:
                self.colored_print(f"{task.id}\t{title_display}\t\t{desc_display}\t\t{status}", 'green')
            else:
                print(f"{task.id}\t{title_display}\t\t{desc_display}\t\t{status}")

    def update_task(self, task_id: int, title: Optional[str] = None,
                   description: Optional[str] = None) -> None:
        """Update a task"""
        try:
            result = self.service.update_task(task_id, title, description)
            if result:
                self.colored_print(f"Task {task_id} updated successfully", 'green')
            else:
                self.colored_print(f"Error: Task with ID {task_id} not found", 'red')
        except ValueError as e:
            self.colored_print(f"Error: {e}", 'red')

    def delete_task(self, task_id: int) -> None:
        """Delete a task"""
        result = self.service.delete_task(task_id)
        if result:
            self.colored_print(f"Task {task_id} deleted successfully", 'red')
        else:
            self.colored_print(f"Error: Task with ID {task_id} not found", 'red')

    def mark_complete(self, task_id: int) -> None:
        """Mark a task as complete"""
        result = self.service.mark_complete(task_id)
        if result:
            self.colored_print(f"Task {task_id} marked as complete", 'green')
        else:
            self.colored_print(f"Error: Task with ID {task_id} not found", 'red')

    def mark_incomplete(self, task_id: int) -> None:
        """Mark a task as incomplete"""
        result = self.service.mark_incomplete(task_id)
        if result:
            self.colored_print(f"Task {task_id} marked as incomplete", 'yellow')
        else:
            self.colored_print(f"Error: Task with ID {task_id} not found", 'red')