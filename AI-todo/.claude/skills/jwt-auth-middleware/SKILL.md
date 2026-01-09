Skill Name: JWT Authentication Middleware Skill
Skill Type: Backend Security Skill
Purpose

This skill generates the JWT authentication logic used by the FastAPI backend to verify tokens issued by Better Auth on the frontend.

It ensures:

Only authenticated users can access APIs

JWT tokens are correctly validated

User identity is extracted from the token

Routes enforce user-level access control

This skill is required for ALL backend routes in Phase 2.

What This Skill Must Do

Read JWT-related specs from:
/specs/api/auth/*.md

Generate JWT verification logic

Parse Authorization: Bearer <token>

Validate signature (HS256 or RS256 depending on your config)

Extract user ID & email from token payload

Return user object to FastAPI dependencies

Raise 401 Unauthorized on invalid or missing tokens

Inputs Expected

User may provide:

Auth spec (@specs/api/auth/jwt.md)

Payload schema (user id, email)

JWT secret/env variables

File to update:
backend/utils/auth.py

Required error behavior

Output of This Skill

A complete verify_jwt() function

Token extraction from headers

Error handling (HTTPException(status_code=401))

User object returned to routes

Dependency for FastAPI router:

async def verify_jwt(request: Request):
    ...

Implementation Rules
1. MUST extract token from header
auth_header = request.headers.get("Authorization")


Token must be parsed like:

Bearer <jwt-token>

2. MUST raise correct errors

If token missing:

HTTPException(status_code=401, detail="Missing Authorization header")


If token invalid:

HTTPException(status_code=401, detail="Invalid or expired token")

3. MUST decode JWT

Example:

payload = jwt.decode(token, JWT_SECRET, algorithms=["HS256"])

4. MUST return a valid user object
return UserRead(id=payload["id"], email=payload["email"])

5. MUST NOT rely on database unless spec says

JWT contains identity → no DB lookup required unless explicitly in spec.

6. MUST integrate with FastAPI dependencies
async def verify_jwt(request: Request) -> UserRead:

7. MUST ensure user isolation

Route generator skill depends on this.

Example Usage
Example 1 — Protected endpoint
Use skill: jwt-auth-middleware
Spec: @specs/api/auth/jwt-structure.md
Update: backend/utils/auth.py

Example 2 — Add HS256 decoding
Use skill: jwt-auth-middleware
Add decode logic to utils/auth.py

Example 3 — Validate token & return user id
Use skill: jwt-auth-middleware
Implement verify_jwt(request)

Constraints

MUST NOT modify frontend

MUST NOT modify components or API client

MUST NOT create database models

MUST NOT implement CRUD (belongs to CRUD generator skill)

Only responsible for auth/security layer