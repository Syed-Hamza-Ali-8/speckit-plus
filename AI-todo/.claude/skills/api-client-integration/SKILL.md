Skill Name: API Client Integration Skill
Skill Type: Frontend → Backend Communication Skill
Purpose

This skill is responsible for generating and updating the frontend API client module inside:

frontend/lib/api.ts


This client enables secure, authenticated communication between:

Next.js frontend (Better Auth JWT)

FastAPI backend

This skill ensures:

Correct API endpoints

JWT attached in headers

TypeScript types

Error handling

Consistent method names

Frontend follows spec-defined API contracts

What This Skill Must Do

Create or update /frontend/lib/api.ts

Read API specs from /specs/api/*

Implement REST calls for:

GET tasks

POST new task

PUT update task

DELETE task

PATCH mark complete

Use fetch with proper headers

Automatically insert JWT from Better Auth

Build type-safe request + response handlers

Inputs Expected by This Skill

User may provide:

API spec file (ex: @specs/api/tasks.md)

New API method to implement

Destination file to update

Response structure from spec

Output of This Skill

A complete API wrapper with:

typed responses

typed params

error handling

Methods like:

api.getTasks()
api.getTask(id)
api.createTask(data)
api.updateTask(id, data)
api.deleteTask(id)
api.markComplete(id)

Implementation Rules
1. MUST follow API spec exactly

Endpoints must match spec

Query params must match spec

Response models must match spec

2. MUST retrieve JWT from Better Auth

Every fetch must include:

Authorization: `Bearer ${token}`


Token must be retrieved via:

const session = await auth();
const token = session?.user?.token;

3. MUST use absolute URLs

The base URL must come from environment:

process.env.NEXT_PUBLIC_API_URL

4. MUST use TypeScript

Types must match spec:

interface Task {
  id: string;
  title: string;
  completed: boolean;
}

5. MUST centralize fetch logic

Using a helper:

async function request(path: string, options: RequestInit = {}) { ... }

6. Error handling

If API returns an error:

throw new Error(`API Error: ${res.status}`);

7. No UI logic

This skill builds the client layer only.

Example Usage
Example 1 — Implement tasks API
Use skill: api-client-integration
Spec: @specs/api/tasks.md
Update: frontend/lib/api.ts

Example 2 — Add markComplete method
Use skill: api-client-integration
Spec: @specs/api/mark-complete.md
Add function: api.markComplete(id)

Example 3 — Update response type
Use skill: api-client-integration
Spec: @specs/api/task-response.md
Modify: Task type in api.ts

Constraints

MUST only edit frontend/lib/api.ts

MUST NOT modify backend code

MUST NOT modify frontend UI

MUST NOT implement components

MUST only handle networking & types