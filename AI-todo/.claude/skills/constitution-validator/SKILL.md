Skill Name: Constitution Validator
Skill Type: System Governance & Compliance Skill
Purpose

This skill enforces the project’s internal Constitution located at:

.specify/memory/constitution.md


Its role is to:

Validate whether a requested action aligns with the Constitution

Detect forbidden behaviors

Prevent agents from generating code outside allowed domains

Ensure strict Spec-Driven Development

Protect monorepo structure and folder boundaries

Prevent accidental or unauthorized system modifications

Produce warnings when a user request violates a rule

This skill acts as the guardian of the system.

What This Skill Must Do
1️⃣ Read & interpret relevant Constitution rules

This includes rules about:

file boundaries

domain separation

spec-driven workflow

subagent scope

user confirmation requirements

governance instructions

2️⃣ Validate actions before they occur

Examples:

Prevent frontend agent from modifying backend

Prevent DB agent from touching UI

Prevent CRUD agent from changing models

Prevent any agent from generating code not described in specs

Prevent edits to Constitution or governance files

Ensure user confirmation when modifying existing files

3️⃣ Return warnings and block tasks when required

If rules are violated, return structured warnings instead of executing tasks.

4️⃣ Run automatically before all subagent tasks

This skill is designed to operate as a gatekeeper.

Inputs Expected

The user or subagent may provide:

Proposed task description

Target file paths

Referenced specs

Requested skill/action

This skill checks those inputs against the Constitution.

Output of This Skill

VALIDATION RESULT:

“Allowed” → Action can proceed

“Denied” → Violates Constitution

“Needs Clarification” → Missing info in spec

Optional warnings

Optional suggested corrections

Example output:

❌ Constitution Violation:
Frontend agent cannot modify backend/routes/tasks.py

Please update your request.

Implementation Rules
1. MUST enforce Constitution boundaries

Examples:

UI code → frontend only

Routes → backend only

DB schema → models only

Auth → utils/auth.py only

specs/ files can’t be overwritten by code

2. MUST check file path validity

Allowed:

frontend/
backend/
specs/
.claude/


Forbidden:

node_modules/
.env*
system/
os/
packages not in repo

3. MUST require confirmation before modifying files
⚠️ Warning: You are about to overwrite an existing file.
Confirm? (yes/no)

4. MUST stop hallucinated files

If a user asks for a file not in spec:

❌ File not defined in any spec.
Please create or update the spec first.

5. MUST validate Spec-Driven behavior

If a request lacks a referenced spec:

⚠️ Missing spec reference.  
This action cannot proceed without @specs/...  

6. MUST prevent agents from modifying Constitution
❌ Constitution modification is prohibited.

Example Usage
Example 1 — User requests disallowed action
Add a new field to Task model (but spec doesn’t mention it)


Skill 9 should respond:

❌ Constitution Violation:
Specs do not authorize modifying Task schema.
Please update @specs/database/task.md first.

Example 2 — User tries to edit backend with frontend skill
Use skill: nextjs-page-generator  
Modify: backend/routes/tasks.py


Skill 9 should respond:

❌ Invalid Domain:
Next.js Page Generator cannot modify backend files.

Example 3 — User asks for unclear change
Implement createTask page.


Skill 9 responds:

⚠️ Missing Spec:
Please provide @specs/ui/pages/create-task.md

Constraints

This skill must NOT:

Modify any project files

Generate code

Write new spec files

Overwrite Constitution

Execute subagent tasks