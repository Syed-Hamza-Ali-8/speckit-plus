# Data Model: Todo Console Application

## Task Entity

**Name**: Task
**Description**: Represents a single todo item that users can manage

### Fields
- **id**: Integer (required, unique, auto-incrementing from 1)
  - Purpose: Unique identifier for each task
  - Constraints: Positive integer, unique across all tasks
- **title**: String (required)
  - Purpose: Brief description of the task
  - Constraints: Non-empty string, maximum 200 characters
- **description**: String (optional)
  - Purpose: Detailed information about the task
  - Constraints: Can be empty, maximum 1000 characters
- **completed**: Boolean (required)
  - Purpose: Indicates whether the task has been completed
  - Constraints: True (completed) or False (incomplete), defaults to False

### Validation Rules
- Title must not be empty or contain only whitespace
- ID must be unique across all tasks in memory
- ID must be a positive integer
- Title and description lengths must not exceed maximums

### State Transitions
- **Initial State**: completed = False (when task is created)
- **Transition 1**: completed = False → completed = True (when task is marked complete)
- **Transition 2**: completed = True → completed = False (when task is marked incomplete)

## Task Collection

**Name**: TaskManager
**Description**: In-memory storage and management system for tasks

### Operations
- **Add Task**: Creates a new task with unique ID and initial state (completed = False)
- **Get All Tasks**: Returns all tasks in the collection
- **Get Task by ID**: Returns a specific task or null if not found
- **Update Task**: Modifies existing task properties
- **Delete Task**: Removes a task from the collection
- **Mark Complete/Incomplete**: Updates the completion status of a task

### Constraints
- All tasks are stored in memory only
- Data is lost when the application terminates
- IDs are assigned sequentially starting from 1