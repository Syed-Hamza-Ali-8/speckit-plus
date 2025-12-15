---
id: 001
title: Notifications System Specification
stage: spec
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: notifications-system
branch: main
user: developer
command: /sp.specify
labels: ["specification", "notifications", "rtk-query", "optimistic-updates"]
links:
  spec: specs/notifications-system/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/notifications-system/spec.md
tests:
  - N/A (specification phase)
---

## Prompt

/sp.specify
## Task 3: Notifications System (10m)
* Notification bell in header (badge count)
* /notifications endpoint → RTK Query
* Mark as read → Optimistic updates
* In-app toast integration

## Response snapshot

Created comprehensive specification for Notifications System feature:

**User Stories (5):**
1. US-1: View Notification Count (badge on bell icon)
2. US-2: View Notifications List (dropdown panel)
3. US-3: Mark Notification as Read (optimistic update)
4. US-4: Mark All as Read (bulk action)
5. US-5: Toast Integration (event-driven toasts)

**Notification Types:**
- task_due, task_overdue, task_completed, welcome, system

**API Endpoints:**
- GET /notifications - Paginated list with unread_count
- PATCH /notifications/{id}/read - Mark single as read
- POST /notifications/mark-all-read - Bulk mark as read
- GET /notifications/unread-count - Lightweight count endpoint

**Key Technical Decisions:**
- Optimistic updates with RTK Query cache manipulation
- Bell badge capped at "9+" for display
- Max 100 notifications per user (auto-cleanup)
- 30-day notification expiry
- Polling (60s) instead of WebSocket (v1 simplicity)

**Files to Create:**
- Backend: notification.py (model, schema, service, routes)
- Frontend: NotificationBell, NotificationDropdown, NotificationItem, notificationApi.ts

## Outcome

- ✅ Impact: Complete specification for in-app notifications with 5 user stories
- 🧪 Tests: Test cases defined for backend and frontend
- 📁 Files: 1 created (spec.md)
- 🔁 Next prompts: /sp.plan to generate implementation plan
- 🧠 Reflection: v1 uses polling over WebSocket for simplicity; future iteration can add real-time push

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to planning phase
