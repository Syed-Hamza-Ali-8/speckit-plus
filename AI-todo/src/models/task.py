"""
Task model and TaskManager for file-based storage
"""
import json
import os
from typing import List, Optional
from dataclasses import dataclass, field, asdict


@dataclass
class Task:
    """
    Represents a single todo item
    """
    id: int
    title: str
    description: Optional[str] = None
    completed: bool = False

    def __post_init__(self):
        """Validate task attributes after initialization"""
        if not self.title or not self.title.strip():
            raise ValueError("Task title cannot be empty or contain only whitespace")

        if len(self.title) > 200:
            raise ValueError("Task title must not exceed 200 characters")

        if self.description and len(self.description) > 1000:
            raise ValueError("Task description must not exceed 1000 characters")

    def to_dict(self):
        """Convert task to dictionary for JSON serialization"""
        return asdict(self)

    @classmethod
    def from_dict(cls, data):
        """Create task from dictionary"""
        return cls(**data)


class TaskManager:
    """
    File-based storage and management system for tasks
    """
    def __init__(self, storage_file="tasks.json"):
        # Use absolute path based on the project root
        import os
        # Get the project root (where this file is located, going up two levels)
        project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        self.storage_file = os.path.join(project_root, storage_file)
        self._tasks: List[Task] = []
        self._next_id = 1
        self.load_tasks()

    def load_tasks(self):
        """Load tasks from file"""
        if os.path.exists(self.storage_file):
            try:
                with open(self.storage_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    self._tasks = [Task.from_dict(task_data) for task_data in data.get('tasks', [])]
                    self._next_id = data.get('next_id', 1)
            except (json.JSONDecodeError, KeyError):
                # If file is corrupted or has wrong format, start fresh
                self._tasks = []
                self._next_id = 1
        else:
            # File doesn't exist, start fresh
            self._tasks = []
            self._next_id = 1

    def save_tasks(self):
        """Save tasks to file"""
        # Prepare data for serialization
        tasks_data = [task.to_dict() for task in self._tasks]
        data = {
            'tasks': tasks_data,
            'next_id': self._next_id
        }

        with open(self.storage_file, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

    def add_task(self, title: str, description: Optional[str] = None) -> Task:
        """Create a new task with unique ID and initial state (completed = False)"""
        task = Task(
            id=self._next_id,
            title=title,
            description=description,
            completed=False
        )
        self._tasks.append(task)
        self._next_id += 1
        self.save_tasks()  # Save after adding
        return task

    def get_all_tasks(self) -> List[Task]:
        """Return all tasks in the collection"""
        return self._tasks.copy()

    def get_task_by_id(self, task_id: int) -> Optional[Task]:
        """Return a specific task or None if not found"""
        for task in self._tasks:
            if task.id == task_id:
                return task
        return None

    def update_task(self, task_id: int, title: Optional[str] = None,
                   description: Optional[str] = None) -> Optional[Task]:
        """Modify existing task properties"""
        task = self.get_task_by_id(task_id)
        if not task:
            return None

        if title is not None:
            if not title or not title.strip():
                raise ValueError("Task title cannot be empty or contain only whitespace")
            if len(title) > 200:
                raise ValueError("Task title must not exceed 200 characters")
            task.title = title

        if description is not None:
            if description and len(description) > 1000:
                raise ValueError("Task description must not exceed 1000 characters")
            task.description = description

        self.save_tasks()  # Save after updating
        return task

    def delete_task(self, task_id: int) -> bool:
        """Remove a task from the collection and reorganize IDs"""
        task = self.get_task_by_id(task_id)
        if task:
            self._tasks.remove(task)

            # Reorganize IDs to fill the gap
            self._reorganize_task_ids()

            self.save_tasks()  # Save after deleting
            return True
        return False

    def _reorganize_task_ids(self):
        """Reorganize task IDs to be sequential without gaps"""
        # Sort tasks by their current ID to maintain order
        self._tasks.sort(key=lambda x: x.id)

        # Assign new sequential IDs starting from 1
        for i, task in enumerate(self._tasks):
            task.id = i + 1

        # Update next_id to be one more than the highest ID
        self._next_id = len(self._tasks) + 1

    def mark_complete(self, task_id: int) -> bool:
        """Mark a task as completed"""
        task = self.get_task_by_id(task_id)
        if task:
            task.completed = True
            self.save_tasks()  # Save after marking complete
            return True
        return False

    def mark_incomplete(self, task_id: int) -> bool:
        """Mark a completed task as incomplete"""
        task = self.get_task_by_id(task_id)
        if task:
            task.completed = False
            self.save_tasks()  # Save after marking incomplete
            return True
        return False