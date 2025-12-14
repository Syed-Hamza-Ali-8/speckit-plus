Skill Name: Tailwind UI Component Builder

Purpose
This skill generates reusable, modular, TailwindCSS-based React UI components strictly based on the UI specs found in:

/specs/ui/components/


It takes UX/UI instructions from a spec → produces polished React components compatible with Next.js App Router.

This skill focuses on frontend UI only, and must not modify backend, database, or API schemas.

What This Skill Must Do

Read UI component specs such as:
@specs/ui/components/task-card.md

Produce reusable React components inside:
frontend/components/*

Follow component design patterns used in the app

Use TailwindCSS classes for styling

Use TypeScript

Support both server & client components (decide based on spec)

Ensure components are:

reusable

accessible

spec-compliant

consistent with design guidelines

Inputs Expected by This Skill

User might provide:

Component spec file
@specs/ui/components/button.md

Component purpose
"Implement a reusable TaskCard component"

Destination path
frontend/components/TaskCard.tsx

Props definition from the spec

Output of This Skill

Generates a complete React component file

Uses Tailwind utility classes only

Includes proper prop types

Handles loading/error/empty states if specified

Uses absolute imports

Uses client/server mode correctly

Implementation Rules
1. Follow Spec-Driven Development

No guessing

No creative additions

Only build what the spec describes

2. Folder Structure

Components must go inside:

frontend/components/


File naming:

TaskCard.tsx

TaskList.tsx

Button.tsx

3. TailwindCSS Only

No inline styling

No external CSS files

Use consistent utility class patterns

4. Props Must Be Typed

Example:

interface TaskCardProps {
  id: string;
  title: string;
  completed: boolean;
}

5. Accessibility Requirements

Buttons must use <button>

Interactive elements must have aria-label when needed

Inputs must have labels

6. Client vs Server Component

If component uses state, events → "use client"

If purely visual → server component

7. Error & Empty States

If defined in spec:

<p className="text-red-600">Failed to load component data.</p>

Example Usage
Example 1 — Create Task Card Component
Use skill: tailwind-ui-component-builder
Spec: @specs/ui/components/task-card.md
Generate: frontend/components/TaskCard.tsx

Example 2 — Generate Button Component
Use skill: tailwind-ui-component-builder
Spec: @specs/ui/components/button.md
Generate: frontend/components/Button.tsx

Example 3 — Update Component
Use skill: tailwind-ui-component-builder
Spec: @specs/ui/components/tag.md
Update: frontend/components/Tag.tsx

Constraints

MUST follow spec exactly

MUST place files in correct frontend folder

MUST NOT modify backend or database

MUST NOT create API calls (belongs to page generator)

MUST write clean maintainable components