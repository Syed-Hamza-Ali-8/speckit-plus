Skill Name: CRUD Logic Generator
Skill Type: Backend Data Access Layer Skill
Purpose

This skill generates:

CRUD service functions

SQLModel queries

Encapsulated data-access logic

Safe DB operations with try/except

Logic that API routes (Skill 11) will call

This skill does NOT define routes, models, or DB sessions.
It only generates reusable functions inside:

backend/services/<model>_service.py

What This Skill Must Do
1️⃣ Read database or API specs

Specs define:

Which operations exist

Required fields

Required validations

Filtering rules

Sorting rules

Relationship updates

Soft-delete vs hard-delete

2️⃣ Generate CRUD service files

For example:

backend/services/task_service.py


Inside, CRUD must include:

get_<model>

list_<model>

create_<model>

update_<model>

delete_<model>

Custom operations (from spec)

e.g., mark_complete

3️⃣ Use SQLModel with Session

All functions accept:

session: Session

4️⃣ Use central error utilities

From:

utils/errors.py


Examples:

from utils.errors import not_found, conflict, bad_request

5️⃣ Return consistent models / Pydantic schemas
6️⃣ Use safe try/except blocks

No unhandled SQLAlchemy exceptions.

7️⃣ Must NOT:

Modify API routes

Modify models

Modify DB schema

Touch frontend

Inputs Expected

The user may provide:

CRUD spec
@specs/api/tasks/crud.md

Model name

File to generate
backend/services/task_service.py

Custom actions spec
@specs/api/tasks/mark-complete.md

Outputs of This Skill

A complete service file under:

backend/services/<model>_service.py


With functions:

create_<model>()

get_<model>()

list_<model>()

update_<model>()

delete_<model>()

Optional custom functions (from spec)

Implementation Rules
1. CRUD Naming Conventions
Operation	Function Name
Create	create_model()
Read by ID	get_model()
List	list_models()
Update	update_model()
Delete	delete_model()

(Where “model” is lowercase singular)

2. SAFE CRUD Pattern

All CRUD must follow this pattern:

try:
    obj = session.get(Model, id)
    if not obj:
        not_found("Model not found")
except SQLAlchemyError:
    error_response("DatabaseError", "Failed to fetch model")

3. Use SQLModel Queries

Example list:

session.exec(select(Task)).all()


Example update:

for key, value in data.dict(exclude_unset=True).items():
    setattr(task, key, value)

4. Handle Relationships Correctly

If spec defines relationships, update them properly.

5. Must follow spec exactly

No extra operations.

Example Usage
Example 1
Use skill: crud-logic-generator
Spec: @specs/api/tasks/crud.md
Generate: backend/services/task_service.py

Example 2
Use skill: crud-logic-generator
Spec: @specs/api/tasks/mark-complete.md
Add mark_complete function

Constraints

This skill MUST NOT:

Generate routes

Generate models

Generate schemas

Perform authentication

Write frontend code

Create migrations

It only creates internal CRUD logic.