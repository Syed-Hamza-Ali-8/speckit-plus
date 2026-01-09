Skill Name: Workflow Engine & State Machine Generator
Skill Type: Advanced Backend Business Logic Skill
Purpose

This skill implements complex workflows defined in Phase 2 specs, such as:

Multi-step task flows

Conditional branching logic

Approval workflows

Timed workflows

Activity pipelines

State transitions (e.g., “pending → active → completed”)

This skill generates workflow logic, NOT routes or CRUD.
APIs will call this logic — not duplicate it.

What This Skill Must Do
1️⃣ Parse workflow specs in:
/specs/workflows/*.md
/specs/api/workflows/*.md
/specs/features/*.md

2️⃣ Generate workflow service files:
backend/workflows/<workflow_name>_workflow.py

3️⃣ The skill must understand:

States

Transitions

Guards (conditions)

Actions (functions to call)

Side effects (DB updates, notifications)

Entry/exit actions

Preconditions

Postconditions

4️⃣ Provide reusable workflow engine helpers:

advance_state()

validate_transition()

current_state()

5️⃣ Integrate with CRUD logic (Skill 12)

But do NOT duplicate CRUD.

6️⃣ Use error utilities from Skill 10.
7️⃣ Follow specification EXACTLY — no invention.
Inputs Expected

User or spec may provide:

Workflow state diagram

Transition rules

Business rules

Process description

Event/action triggers

Step-by-step instructions

Multi-step API spec

Example:

@specs/workflows/task-lifecycle.md

Outputs of This Skill

A fully implemented workflow file:

backend/workflows/task_lifecycle_workflow.py


Containing:

State enum

Transition matrix

Guards

Actions

Workflow runner

Validation logic

Implementation Rules
1. State Machine MUST be explicit

States defined using Enum:

class TaskState(str, Enum):
    pending = "pending"
    in_progress = "in_progress"
    completed = "completed"
    archived = "archived"

2. Transitions MUST follow spec

Stored in a matrix:

TRANSITIONS = {
    TaskState.pending: [TaskState.in_progress],
    TaskState.in_progress: [TaskState.completed, TaskState.pending],
    TaskState.completed: [TaskState.archived],
}

3. Guard functions MUST follow spec
def can_start(task):
    return task.assigned_to is not None

4. Action functions MUST call CRUD

Example:

task = task_crud.update_task(...)

5. Workflow runner MUST be generic
def advance_state(task, to_state, session):
    validate_transition(task.state, to_state)
    task.state = to_state
    session.add(task)
    session.commit()
    return task

6. MUST reject invalid transitions
❌ Invalid transition: in_progress → pending

7. MUST use error handler skill
Example Usage
Example 1 — Simple state transition
Use skill: workflow-engine
Spec: @specs/workflows/task-lifecycle.md


→ Generates:

backend/workflows/task_lifecycle_workflow.py

Example 2 — Multi-step onboarding flow
Use skill: workflow-engine
Spec: @specs/workflows/onboarding.md

Constraints

This skill MUST NOT:

Touch UI

Touch frontend files

Modify API routes

Modify DB models

Modify CRUD logic

Invent transitions not in spec

This skill ONLY generates workflow/state logic.