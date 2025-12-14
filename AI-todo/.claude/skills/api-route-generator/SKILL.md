Skill Name: API Route Generator
Skill Type: Backend API Generation Skill
Purpose

This skill generates FastAPI route handlers based on API specs found in:

/specs/api/*.md


It turns spec definitions → fully working FastAPI endpoint code that follows:

REST conventions

OpenAPI documentation

SQLModel usage

Auth requirements

Error handling conventions

CRUD naming rules

Response model standards

Project folder structure

What This Skill Must Do
1️⃣ Read API specs

Spec structure includes:

Endpoint

Method (GET, POST, PATCH…)

Params

Request body

Response model

Auth requirement

Error cases

Description

2️⃣ Generate API route file in:
backend/routes/<resource>.py

3️⃣ Use:

SQLModel for models

Session for DB access

Error-handling helpers from utils/errors.py

Auth middleware from utils/auth.py

Pydantic models for request/response

4️⃣ Must create CRUD functions based on spec

Examples:

listTasks

getTask

createTask

updateTask

deleteTask

markComplete

etc.

5️⃣ MUST NOT guess missing behaviors

If spec is unclear → ask for clarification.

6️⃣ MUST follow your monorepo structure

Frontend excluded; this skill is backend-only.

Inputs Expected

User may provide:

API spec reference:
@specs/api/tasks/create-task.md

File to generate:
backend/routes/tasks.py

Partial spec excerpt

A new endpoint description

Outputs of This Skill

Fully implemented FastAPI route handler(s)

Includes:

Correct imports

Input models

Output models

Route decorators

Auth enforcement

Error handling

DB session usage

CRUD logic with try/except safety

Implementation Rules
1. Use FastAPI Router per resource
router = APIRouter(prefix="/tasks", tags=["tasks"])

2. Use Pydantic request body models from specs
class CreateTaskRequest(BaseModel):
    title: str
    description: str | None = None

3. Session dependency
session: Session = Depends(get_session)

4. Use shared error handlers
from utils.errors import not_found, bad_request

5. Use Auth when required
if spec.auth_required:
    user = auth.require_user(session=session)

6. Use response_model everywhere
@router.get("/", response_model=list[TaskRead])

7. No business logic outside specs

If spec doesn’t define a field → do not invent it.

Example Usage
Example 1
Use skill: api-route-generator
Spec: @specs/api/tasks/list.md
Generate: backend/routes/tasks.py

Example 2
Add new PATCH /tasks/{id}/complete from @specs/api/complete.md

Constraints

This skill must NOT:

Generate database models

Generate frontend code

Write non-spec endpoints

Modify unrelated routes

Guess missing fields

This skill only writes backend route code defined in specs.