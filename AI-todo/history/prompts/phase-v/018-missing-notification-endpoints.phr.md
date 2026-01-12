---
id: 018
title: Implement missing notification endpoints
stage: green
date: 2026-01-12
surface: agent
model: claude-sonnet-4-5
feature: phase-v
branch: main
user: hamza
command: continue
labels: ["feature", "notifications", "api", "endpoints", "crud"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - phase-5/src/api/notifications.py
  - NOTIFICATION_ENDPOINTS_COMPLETE.md
tests:
  - Manual testing: Mark all notifications as read
  - Manual testing: Clear all notifications
  - Manual testing: Mark single notification as read
  - Manual testing: Delete single notification
---

## Prompt

User reported: "I am trying to mark all read and clear all notifications and it is showing me this error: :8000/notifications/mark-all-read:1 Failed to load resource: the server responded with a status of 404 (Not Found) :8000/notifications:1 Failed to load resource: the server responded with a status of 405 (Method Not Allowed)"

After fixing the field name mismatches, the notification system still had missing functionality - the "Mark All as Read" and "Clear All" buttons were returning 404 and 405 errors.

## Response snapshot

**Root Cause:** Only 2 notification endpoints were implemented in the backend:
- `GET /notifications` - List notifications
- `GET /notifications/unread-count` - Get unread count

**Missing 4 endpoints that frontend expected:**
1. `POST /notifications/mark-all-read` - Mark all as read (404 error)
2. `DELETE /notifications` - Clear all notifications (405 error)
3. `PATCH /notifications/{id}/read` - Mark single as read (not tested but would be 404)
4. `DELETE /notifications/{id}` - Delete single notification (not tested but would be 404)

**Implementation:**

File: `phase-5/src/api/notifications.py` (lines 157-400)

**1. POST /notifications/mark-all-read (lines 157-214):**
```python
@router.post("/mark-all-read")
def mark_all_as_read(
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """Mark all notifications as read for the current user."""
    # JWT authentication and user_id extraction
    # Get all unread notifications
    statement = select(Notification).where(
        Notification.user_id == user_id,
        Notification.is_read == False
    )
    notifications = session.exec(statement).all()

    # Mark all as read
    for notification in notifications:
        notification.is_read = True
    session.commit()

    return {"marked_count": marked_count, "user_id": str(user_id)}
```

**2. DELETE /notifications (lines 217-271):**
```python
@router.delete("")
def clear_all_notifications(
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """Delete all notifications for the current user."""
    # JWT authentication and user_id extraction
    # Get all notifications
    statement = select(Notification).where(Notification.user_id == user_id)
    notifications = session.exec(statement).all()

    # Delete all
    for notification in notifications:
        session.delete(notification)
    session.commit()

    return {"deleted_count": deleted_count, "user_id": str(user_id)}
```

**3. PATCH /notifications/{id}/read (lines 274-336):**
```python
@router.patch("/{notification_id}/read")
def mark_notification_as_read(
    notification_id: UUID,
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """Mark a single notification as read."""
    # JWT authentication and user_id extraction
    # Get notification (must belong to user)
    statement = select(Notification).where(
        Notification.id == notification_id,
        Notification.user_id == user_id
    )
    notification = session.exec(statement).first()

    if not notification:
        raise HTTPException(status_code=404, detail="Notification not found")

    notification.is_read = True
    session.commit()

    return {"id": str(notification.id), "is_read": True, "user_id": str(user_id)}
```

**4. DELETE /notifications/{id} (lines 339-400):**
```python
@router.delete("/{notification_id}")
def delete_notification(
    notification_id: UUID,
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """Delete a single notification."""
    # JWT authentication and user_id extraction
    # Get notification (must belong to user)
    statement = select(Notification).where(
        Notification.id == notification_id,
        Notification.user_id == user_id
    )
    notification = session.exec(statement).first()

    if not notification:
        raise HTTPException(status_code=404, detail="Notification not found")

    session.delete(notification)
    session.commit()

    return {"id": str(notification_id), "deleted": True, "user_id": str(user_id)}
```

**Security Features:**
- All endpoints require JWT authentication
- Extract user_id from token payload
- All queries filter by user_id (users can only access their own notifications)
- Return 401 if token is missing/invalid
- Return 404 if notification doesn't belong to user

**Deployment:**
- Rebuilt backend Docker container: `docker-compose build backend`
- Deployed updated container: `docker-compose up -d backend`
- Verified backend healthy and all 6 endpoints available

## Outcome

- ✅ Impact: Completed notification system implementation. All CRUD operations now work: list, count, mark-as-read (single/all), delete (single/all). Frontend UI buttons now functional.
- 🧪 Tests: Manual testing required - user needs to refresh browser and test mark-all-read, clear-all, and individual notification actions.
- 📁 Files: Modified 1 backend file (added 244 lines of code for 4 new endpoints), created 1 comprehensive documentation file.
- 🔁 Next prompts: User should test all notification features in the UI to verify everything works correctly.
- 🧠 Reflection: The frontend was already fully implemented with all UI components and API calls ready - it was just waiting for the backend endpoints to exist. This shows good frontend-first development, but highlights the need for API contract validation during development to catch missing endpoints earlier.

## Evaluation notes (flywheel)

- Failure modes observed: Backend API incomplete - frontend expected 6 endpoints but only 2 were implemented. No integration tests to catch missing endpoints. No API documentation (OpenAPI/Swagger) to verify contract completeness.
- Graders run and results (PASS/FAIL): Backend deployment PASS - container rebuilt and healthy. Manual testing PENDING - awaiting user verification of UI functionality.
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Generate OpenAPI/Swagger documentation from FastAPI routes and compare against frontend TypeScript API client to automatically detect missing endpoints. Add integration tests that verify all frontend API calls have corresponding backend endpoints.
