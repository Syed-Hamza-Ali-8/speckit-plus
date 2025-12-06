# CLI Contracts: Todo Console Application

## Command Interface Specifications

### Add Task Command
**Command**: `add <title> [description]`
**Description**: Creates a new task with the provided title and optional description
**Parameters**:
- `title`: Required string (task title)
- `description`: Optional string (task description)
**Output**: Success message with assigned task ID
**Error Cases**:
- Empty title: Returns error message
- Title too long: Returns error message

### List Tasks Command
**Command**: `list` or `view`
**Description**: Displays all tasks with their details and completion status
**Parameters**: None
**Output**: Formatted list of all tasks showing ID, title, description, and status
**Error Cases**: None (shows message if no tasks exist)

### Update Task Command
**Command**: `update <id> <title> [description]`
**Description**: Updates an existing task with new title and optional description
**Parameters**:
- `id`: Required integer (task identifier)
- `title`: Required string (new task title)
- `description`: Optional string (new task description)
**Output**: Success confirmation message
**Error Cases**:
- Invalid ID: Returns error message
- Non-existent task: Returns error message
- Empty title: Returns error message

### Delete Task Command
**Command**: `delete <id>`
**Description**: Removes a task from the system
**Parameters**:
- `id`: Required integer (task identifier)
**Output**: Success confirmation message
**Error Cases**:
- Invalid ID: Returns error message
- Non-existent task: Returns error message

### Mark Complete Command
**Command**: `complete <id>`
**Description**: Marks a task as completed
**Parameters**:
- `id`: Required integer (task identifier)
**Output**: Success confirmation message
**Error Cases**:
- Invalid ID: Returns error message
- Non-existent task: Returns error message

### Mark Incomplete Command
**Command**: `incomplete <id>`
**Description**: Marks a completed task as incomplete
**Parameters**:
- `id`: Required integer (task identifier)
**Output**: Success confirmation message
**Error Cases**:
- Invalid ID: Returns error message
- Non-existent task: Returns error message

## Data Format Contracts

### Task Representation
```json
{
  "id": 1,
  "title": "string (required)",
  "description": "string (optional)",
  "completed": "boolean (required)"
}
```

### Success Response Format
```json
{
  "status": "success",
  "message": "string",
  "data": "object (optional)"
}
```

### Error Response Format
```json
{
  "status": "error",
  "message": "string",
  "error_code": "string"
}
```