# Specification — Todo In-Memory Console App (Phase I)

## Feature Overview
Implement a Python console application capable of managing todos in memory.

## Functional Specifications

### 1. Add Task
- User enters a title
- User enters a description
- App stores a task with a unique ID
- Default status: pending

### 2. View Tasks
- Display all tasks
- Show:
  - ID
  - Title
  - Description
  - Status (pending/completed)

### 3. Update Task
- User selects task by ID
- User can update:
  - Title
  - Description

### 4. Delete Task
- User selects task by ID
- Task is removed from memory

### 5. Mark Complete / Incomplete
- User selects task by ID
- Toggle status between:
  - pending
  - completed

## Constraints
- All data is stored in app memory
- No external storage
- Console-based UI
- Use Python modules under `/src`

## Acceptance Criteria
- App runs using a single command: `python -m todo`
- All five features work correctly
- No crashes when invalid IDs are entered
- Code generated must follow clean code patterns
