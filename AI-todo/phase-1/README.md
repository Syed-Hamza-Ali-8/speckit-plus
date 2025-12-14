# Todo Console Application

A command-line todo application that stores tasks in memory with basic CRUD functionality. The application supports 5 core operations: Add, View, Update, Delete, and Mark Complete/Incomplete tasks.

## Features

- Add new tasks with title and description
- View all tasks with status indicators
- Update existing task details
- Delete tasks by ID
- Mark tasks as complete/incomplete

## Prerequisites

- Python 3.13+

## Setup

1. Clone the repository
2. Navigate to the project directory

## Usage

```bash
# Add a task
python src/main.py add "Task Title" "Optional Description"

# View all tasks
python src/main.py list

# Update a task
python src/main.py update 1 "New Title" "New Description"

# Delete a task
python src/main.py delete 1

# Mark task as complete
python src/main.py complete 1

# Mark task as incomplete
python src/main.py incomplete 1
```