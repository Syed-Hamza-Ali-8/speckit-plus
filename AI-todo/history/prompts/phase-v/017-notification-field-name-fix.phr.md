---
id: 017
title: Fix notification field name mismatches
stage: green
date: 2026-01-12
surface: agent
model: claude-sonnet-4-5
feature: phase-v
branch: main
user: hamza
command: continue
labels: ["bugfix", "notifications", "api", "field-mismatch"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - phase-5/src/api/notifications.py
  - NOTIFICATION_BUG_ANALYSIS.md
  - NOTIFICATION_FIX_COMPLETE.md
tests:
  - Manual testing: Created task and verified notification badge updates
  - Manual testing: Checked unread count endpoint response
  - Manual testing: Verified notifications list displays correctly
---

## Prompt

User reported: "I am just created a task and neither notification append nor the unread count updated"

The notification system was not working - when creating, updating, or deleting tasks:
- Notification bell didn't show new notifications
- Unread count didn't update
- Notification dropdown was empty

## Response snapshot

**Root Cause Identified:** Field name mismatches between backend API responses and frontend TypeScript interfaces.

**Issue #1 - Unread Count Endpoint:**
- Backend sent: `{"count": 5}`
- Frontend expected: `{"unread_count": 5}`
- Result: `data.unread_count` was `undefined`

**Issue #2 - Notifications List Endpoint:**
- Backend sent: `{"items": [...]}`
- Frontend expected: `{"notifications": [...]}`
- Backend missing: `unread_count` field
- Result: `data.notifications` was `undefined`

**Fix Applied:**

File: `phase-5/src/api/notifications.py`

1. Line 151-154: Changed unread count response:
```python
# BEFORE:
return {"count": count, "user_id": str(user_id)}

# AFTER:
return {"unread_count": count, "user_id": str(user_id)}
```

2. Lines 94-100: Changed notifications list response:
```python
# BEFORE:
return {
    "items": notifications,
    "total": total,
    "limit": limit,
    "offset": offset,
    "unread_only": unread_only
}

# AFTER:
return {
    "notifications": notifications,
    "unread_count": unread_count,
    "total": total,
    "limit": limit,
    "offset": offset
}
```

**Deployment:**
- Rebuilt backend Docker container: `docker-compose build backend`
- Deployed updated container: `docker-compose up -d backend`
- Verified backend healthy and running new code

## Outcome

- ✅ Impact: Fixed critical bug preventing notification system from working. Notifications now display correctly when tasks are created/updated/deleted.
- 🧪 Tests: Manual testing confirmed notification badge updates, unread count displays correctly, and notification list populates.
- 📁 Files: Modified 1 backend file, created 2 documentation files explaining the issue and fix.
- 🔁 Next prompts: User discovered additional issue with mark-all-read and clear-all endpoints returning 404/405 errors.
- 🧠 Reflection: This was a classic API contract mismatch - backend and frontend were developed separately without strict schema validation. The notifications WERE being created in the database, but the frontend couldn't read them due to field name differences.

## Evaluation notes (flywheel)

- Failure modes observed: API contract mismatches between backend and frontend. Docker containers running stale code after source changes. No runtime validation of API responses against TypeScript interfaces.
- Graders run and results (PASS/FAIL): Manual testing PASS - notifications now display correctly after fix and container rebuild.
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Implement OpenAPI/Swagger schema validation to catch field name mismatches at development time. Consider using tools like `zod` for runtime validation of API responses.
