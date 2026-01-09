# SKILL.md — Background Job Queue & Async Task Runner
Skill Name: Background Job Queue & Async Task Runner
Skill Type: Backend Infrastructure Skill

## Purpose
This skill handles asynchronous tasks, scheduled jobs, and long-running operations in Phase 2 workflows. It ensures heavy operations do not block the main workflow and supports tasks like email sending, notifications, or data processing in the background.

## What This Skill Must Do
1️⃣ Provide an async task runner
- Run background jobs safely
- Queue tasks and prioritize them
- Retry failed tasks

2️⃣ Schedule periodic jobs
- Cron-style scheduling
- Delayed execution

3️⃣ Integrate with other skills
- Workflow Engine (Skill 13)
- Notification system (Skill 14)
- Error Handling (Skill 10)

4️⃣ Task templates
- Provide reusable job templates
- Standardize async task execution

## Inputs Expected
- Task function / job
- Schedule / delay
- Retry rules
- Dependencies (optional)
- Workflow triggers (optional)

## Outputs of This Skill
- Async runner utility
- Job templates
- Scheduled job scripts
- Integration hooks for workflows

## Implementation Rules
1. MUST use asyncio or compatible async framework
2. MUST provide centralized error handling
3. MUST allow retry on failure
4. MUST NOT:
   - Modify CRUD logic
   - Handle frontend
   - Invent tasks not defined in spec
