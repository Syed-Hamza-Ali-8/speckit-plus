Skill Name: Next.js Page Generator
Purpose

This skill is responsible for generating Next.js App Router pages strictly following the UI specifications inside the /specs/ui/ directory.
It converts UI specs → working pages, layouts, and React server components.

What This Skill Must Do

Read relevant specs provided by the user (e.g., @specs/ui/pages.md, @specs/ui/components.md)

Generate Next.js App Router pages inside /frontend/app/...

Use React Server Components by default

Use TailwindCSS for styling

Use TypeScript

Use Better Auth session hooks where needed

Use the API client functions (from /frontend/lib/api.ts)

Inputs Expected by This Skill

The user may provide:

A reference to a spec file: @specs/ui/pages.md

A specific page name or purpose: "Implement the task list page"

A file path to modify: frontend/app/tasks/page.tsx

Output of This Skill

Create or update Next.js page files

Use useClient only when needed

Import API client using:

import { api } from "@/lib/api";


Produce fully functional UI code

Implementation Rules

This skill must follow:

1. Spec-Driven Development

Only generate what is written in the spec

If something is unclear, request spec clarification

No guessing or inventing UI elements

2. File Structure Requirements

Pages must follow:

frontend/app/tasks/page.tsx
frontend/app/tasks/[id]/page.tsx
frontend/app/layout.tsx
frontend/components/TaskCard.tsx

3. Preferred Patterns

Use server components for simple pages

Use client components only when required (forms, interactivity)

Use Tailwind utility classes

Use semantic HTML

4. Error Handling

If API fails, show simple UI error:

<p className="text-red-600">Failed to load tasks.</p>

5. Authentication

For protected pages:

import { auth } from "@/lib/auth";

const session = await auth();
if (!session) redirect("/login");

6. No Inline Styling

Only Tailwind.

7. Import Paths

Use absolute imports:

import { TaskCard } from "@/components/task-card";

Example Usage (How Claude Should Use This Skill)
Example 1 — Create Task List Page
Use skill: nextjs-page-generator  
Spec: @specs/ui/pages/task-list.md  
Goal: Implement frontend/app/tasks/page.tsx

Example 2 — Add Task Creation Form
Use skill: nextjs-page-generator  
Spec: @specs/ui/pages/create-task.md  
Generate frontend/app/tasks/new/page.tsx

Example 3 — Edit Task Page
Use skill: nextjs-page-generator  
Spec: @specs/ui/pages/edit-task.md  
Path: frontend/app/tasks/[id]/edit/page.tsx

Constraints

MUST follow specs exactly — no extra features

MUST follow monorepo folder structure

MUST follow CLAUDE.md conventions

MUST NOT create backend code (belongs to backend agent)

MUST NOT generate migrations or SQLModel (belongs to database agent)