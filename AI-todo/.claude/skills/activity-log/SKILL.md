# SKILL.md — Activity Log / Audit Log Skill
Skill Name: Activity Log / Audit Log
Skill Type: Backend Infrastructure Skill

## Purpose
This skill provides a standardized system to log all user actions, system events, and workflow transitions. It ensures traceability, auditing, and debugging for Phase 2 multi-agent workflows.

## What This Skill Must Do
1️⃣ Log all actions:
- User actions (CRUD, login/logout)
- System actions (async tasks, notifications)
- Workflow transitions

2️⃣ Maintain structured logs:
- Timestamps
- Actor (user/system)
- Action type
- Resource affected

3️⃣ Support audit retrieval:
- Query logs by user, resource, action
- Filter by time range
- Paginate results

4️⃣ Integrate with other skills:
- Workflow Engine (Skill 13)
- Async Task Runner (Skill 15)
- Notification System (Skill 14)

## Inputs Expected
- Action type
- Actor ID
- Resource details (ID, type)
- Metadata (optional)
- Timestamp (optional, defaults to current)

## Outputs of This Skill
- Activity log entries
- Audit reports (queryable)
- Optional log files for external monitoring

## Implementation Rules
1. MUST provide a centralized logger utility
2. MUST be async-safe
3. MUST integrate with database for persistence
4. MUST NOT:
   - Modify workflows directly
   - Handle frontend
   - Invent actions not defined in spec
