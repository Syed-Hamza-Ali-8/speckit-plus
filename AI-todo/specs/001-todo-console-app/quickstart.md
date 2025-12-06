# Quickstart Guide: Todo Console Application

## Prerequisites
- Python 3.13+ installed
- UV package manager installed (if using dependencies)

## Setup
1. Clone the repository
2. Navigate to the project directory
3. Install dependencies (if any): `uv sync` (though this project uses only built-in libraries)

## Running the Application
```bash
python src/main.py
```

## Available Commands

### Add a Task
```bash
python src/main.py add "Task Title" "Optional Description"
```
Example:
```bash
python src/main.py add "Buy groceries" "Milk, bread, eggs"
```

### View All Tasks
```bash
python src/main.py list
```
or
```bash
python src/main.py view
```

### Update a Task
```bash
python src/main.py update <task_id> "New Title" "New Description"
```
Example:
```bash
python src/main.py update 1 "Updated title" "Updated description"
```

### Delete a Task
```bash
python src/main.py delete <task_id>
```
Example:
```bash
python src/main.py delete 1
```

### Mark Task Complete/Incomplete
```bash
python src/main.py complete <task_id>    # Mark as complete
python src/main.py incomplete <task_id>  # Mark as incomplete
```
Example:
```bash
python src/main.py complete 1
```

## Example Workflow
1. Add tasks:
   ```bash
   python src/main.py add "Finish report"
   python src/main.py add "Buy groceries" "Milk, bread, eggs"
   ```

2. View all tasks:
   ```bash
   python src/main.py list
   ```

3. Mark a task complete:
   ```bash
   python src/main.py complete 1
   ```

4. Update a task:
   ```bash
   python src/main.py update 2 "Buy groceries and cook dinner" "Milk, bread, eggs, chicken"
   ```

5. View updated tasks:
   ```bash
   python src/main.py list
   ```

## Error Handling
- If you try to operate on a non-existent task ID, the application will show an appropriate error message
- If you provide invalid command syntax, the application will show usage instructions