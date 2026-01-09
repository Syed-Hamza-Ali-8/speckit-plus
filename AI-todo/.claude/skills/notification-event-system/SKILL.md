Skill Name: Notification & Event System
Skill Type: Backend Infrastructure Skill
Purpose

This skill provides a standardized way to:

Emit backend events

Listen to events

Trigger workflow actions

Trigger notifications

Send emails (stubbed or real provider)

Send in-app notifications

Create event logs

Glue workflows + CRUD + routes together via events

This is the backbone for multi-step processes in Phase 2.

What This Skill Must Do
1️⃣ Implement an event bus

A simple event emitter allowing:

event_bus.emit("task.completed", payload)
event_bus.on("task.completed", handler)

2️⃣ Generate event handlers based on specs

Specs located in:

/specs/events/*.md
/specs/workflows/*.md
/specs/features/*.md


Example events from specs:

task.created

task.completed

task.assigned

user.registered

3️⃣ Support 2 types of notifications:

System events (internal handlers)

User notifications (email / in-app)

4️⃣ Create notification service:
backend/notifications/notification_service.py

5️⃣ Create a central event bus:
backend/events/event_bus.py

6️⃣ Build integrations with workflows

Event handlers may trigger workflow transitions.

Inputs Expected

User or spec may provide:

Event list

Trigger descriptions

Notification rules

Email templates

In-app notification spec

Workflow triggers

Example event flows

Outputs of This Skill

event_bus.py

notification_service.py

Event handler file(s):

backend/events/handlers/<domain>_handlers.py


Optional email templates inside:

backend/notifications/templates/*.md

Implementation Rules
1. MUST use publish-subscribe pattern
class EventBus:
    listeners = {}

2. MUST validate event names

Only events declared in spec can be used.

3. MUST provide async-safe handlers

Handlers run safely even if slow.

4. MUST use centralized error handler skill
5. MUST NOT:

Touch frontend

Create database logic

Modify CRUD functions

Modify workflows

Invent events not in spec

Add email providers unless spec says so

6. MUST fallback to console printing for emails

Unless spec defines actual email provider.

🔥 Event Flow Example (Skill Responsibilities)

Example from spec:

When a task is completed:
    → Trigger event "task.completed"
    → Workflow moves task to completed state
    → Send email to assigned user
    → Create in-app notification


Skill 14 must generate:

Event listener

Workflow trigger call

Notification trigger

Example Usage
Example 1
Use skill: notification-event-system
Spec: @specs/events/task-events.md
Generate event bus + handlers

Example 2
Use skill: notification-event-system
Add email notification to "task.completed"

Constraints

Skill MUST NOT:

Create routes

Create CRUD

Modify DB schema

Create frontend UI

Guess missing event behavior

Emit events not defined in spec

This skill ONLY builds the event + notification infrastructure.