Skill Name: FastAPI CRUD Route Generator
Skill Type: Backend API Implementation Skill
Purpose

This skill is responsible for generating FastAPI CRUD routes based strictly on the API specifications found in:

/specs/api/


This skill produces:

Route functions

Request/response models

Dependency injection

Database queries (SQLModel ORM)

JWT validation

Proper error handling

This skill enables the backend service to expose REST endpoints for tasks and users, following Phase-2 requirements.

What This Skill Must Do

Read API spec files (@specs/api/*.md)

Implement REST endpoints using FastAPI

Use SQLModel ORM for DB

Validate JWT from Authorization header

Ensure proper user isolation (user can only access own tasks)

Generate error responses (400/401/404)

Produce type-safe request/response models

Register routes into FastAPI app

Inputs Expected by This Skill

User may provide:

Endpoint spec reference
@specs/api/task-create.md

File to update
backend/routes/tasks.py

Request body schema

Response body schema

Required status codes

Output of This Skill

Creates or updates FastAPI routes

Creates Pydantic models as needed

Implements CRUD logic using SQLModel

Adds user authentication logic

Adds database query functions

Updates router imports/registration

Implementation Rules
1. MUST follow API spec exactly

Endpoint paths

Method type (GET/POST/PUT/DELETE/PATCH)

Response model

Error conditions

2. MUST validate JWT

Each route must include:

from utils.auth import verify_jwt
user = verify_jwt(request)

3. MUST enforce user isolation

When fetching tasks:

query = select(Task).where(Task.user_id == user.id)

4. MUST use SQLModel

Example:

task = Task.model_validate(payload)
session.add(task)
session.commit()

5. MUST use async where possible

All routes use:

@router.get(...)
async def get_tasks(...):

6. MUST return proper status codes
Code	Condition
200	OK
201	Created
400	Invalid input
401	Missing/invalid token
404	Resource not found
7. No frontend changes

This skill updates backend only.

Example Usage
Example 1 — Create Task Route
Use skill: fastapi-crud-generator
Spec: @specs/api/tasks-create.md
Generate: backend/routes/tasks.py

Example 2 — Update Task Route
Use skill: fastapi-crud-generator
Spec: @specs/api/tasks-update.md
Modify: backend/routes/tasks.py

Example 3 — Generate response model
Use skill: fastapi-crud-generator
Spec: @specs/api/task-response.md
Add Pydantic model in backend/models/task.py

Constraints

MUST NOT modify frontend code

MUST NOT modify Next.js

MUST NOT invent endpoints

MUST use SQLModel

MUST place routes inside backend/routes/

MUST follow folder structure:

backend/
  routes/
  models/
  db/
  utils/