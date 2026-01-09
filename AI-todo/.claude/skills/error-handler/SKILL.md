Skill Name: Backend Error Handling & Exception Normalization
Skill Type: Backend Infrastructure Skill
Purpose

This skill ensures consistent error handling across the backend by:

Creating shared error response utilities

Normalizing exceptions into stable API responses

Adding global FastAPI exception handlers

Wrapping CRUD logic with safe error patterns

Enforcing consistent response shapes

This skill prevents the backend from throwing raw SQL or Python errors to users.

What This Skill Must Do
1️⃣ Create a global errors.py module

Includes:

HTTPException wrappers

Standard error response models

Normalized messages

Database error translators

2️⃣ Create global exception handlers in FastAPI

Handles:

ValidationError

SQLAlchemyException

NoResultFound

IntegrityError

AuthenticationError

AuthorizationError

Unknown exceptions

3️⃣ Provide safe try/except templates for CRUD
4️⃣ Enforce standard error format:
{
  "error": {
    "type": "NotFoundError",
    "message": "Task not found",
    "detail": null
  }
}

5️⃣ Ensure no sensitive data is ever leaked
Inputs Expected

User or agent may request:

“Generate error module”

“Add consistent error handling to routes”

“Normalize DB exceptions”

“Add global exception handler file”

“Update routes to use error wrappers”

Outputs of This Skill

backend/utils/errors.py

backend/utils/exception_handlers.py

Optional route updates (via other skills, not here)

This skill DOES NOT generate routes — it only creates error-handling structures.

Implementation Rules
1. MUST NOT modify business logic

This is infrastructure-only.

2. MUST create reusable error helpers

Examples:

def not_found(message: str):
    raise HTTPException(status_code=404, detail=message)

3. MUST produce clean error messages

No Python tracebacks
No SQL errors
No debugging text

4. MUST integrate with FastAPI app

Example:

from .exception_handlers import add_exception_handlers
add_exception_handlers(app)

5. MUST avoid circular imports
6. MUST support custom exceptions

Examples:

AuthenticationError

AuthorizationError

DatabaseError

7. MUST ensure JSON-safe errors
Example Usage
Example 1 — Normalize DB errors
Use skill: error-handler
Generate: backend/utils/errors.py

Example 2 — Add global handlers
Use skill: error-handler
Generate: backend/utils/exception_handlers.py

Example 3 — Protect route
Wrap updateTask route with safe CRUD try/except

Constraints

This skill must NOT:

write business logic

write API endpoints

write SQLModel models

write CRUD logic

create database schemas

It ONLY standardizes and structures error behavior.