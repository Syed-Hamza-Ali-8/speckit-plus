# Data Model — Phase 3: Todo AI Chatbot

## New Entities

### 1. ChatSession (In-Memory Only)

```python
class ChatSession:
    """Session state for chat conversations.

    Stored in-memory, not persisted to database.
    Expires after 30 minutes of inactivity.
    """
    session_id: UUID           # Primary key
    user_id: UUID              # FK to users table (for auth)
    created_at: datetime       # Session creation time
    last_activity: datetime    # Last message time (for TTL)
    context: SessionContext    # Conversation context
```

### 2. SessionContext (Embedded in ChatSession)

```python
class SessionContext:
    """Conversation context within a session."""
    messages: list[ChatMessage]           # Last 20 messages (rolling)
    last_intent: str | None               # Most recent intent classification
    last_task_ids: list[UUID]             # Recently referenced tasks
    pending_confirmation: Confirmation | None  # Awaiting user confirmation
```

### 3. ChatMessage (Value Object)

```python
class ChatMessage:
    """Individual message in conversation history."""
    role: Literal["user", "assistant"]
    content: str
    timestamp: datetime
    metadata: MessageMetadata | None
```

### 4. MessageMetadata (Value Object)

```python
class MessageMetadata:
    """Metadata attached to assistant messages."""
    intent: str
    actions: list[ActionTaken]
    processing_time_ms: int
    model: str
```

### 5. Confirmation (Value Object)

```python
class Confirmation:
    """Pending destructive action awaiting user confirmation."""
    action: Literal["delete", "bulk_update"]
    target_ids: list[UUID]
    prompt: str               # What was shown to user
    expires_at: datetime      # Auto-clear after 5 minutes
```

### 6. ActionTaken (Value Object)

```python
class ActionTaken:
    """Record of MCP tool invocation."""
    tool: str                 # list_tasks, create_task, etc.
    success: bool
    summary: str              # Human-readable result summary
    duration_ms: int
```

---

## Relationships

```
┌──────────────────────────────────────────────────────────────┐
│                    In-Memory Session Store                    │
│                                                              │
│  ┌─────────────────┐                                        │
│  │   ChatSession   │                                        │
│  │  session_id (PK)│───────────────────────────────────────┐│
│  │  user_id (FK)   │───────┐                               ││
│  │  context        │       │                               ││
│  └─────────────────┘       │                               ││
│         │                  │                               ││
│         │ 1:1              │                               ││
│         ▼                  │                               ││
│  ┌─────────────────┐       │                               ││
│  │ SessionContext  │       │                               ││
│  │  messages[]     │       │                               ││
│  │  last_task_ids[]│───────┼───────────────────────────────┼┤
│  │  pending_confirm│       │                               ││
│  └─────────────────┘       │                               ││
│                            │                               ││
└────────────────────────────┼───────────────────────────────┼┘
                             │                               │
                             │ FK                            │ FK
                             ▼                               ▼
┌────────────────────────────────────────────────────────────────┐
│                      Database (Neon)                            │
│                                                                 │
│  ┌─────────────────┐        ┌─────────────────┐                │
│  │     users       │        │     tasks       │                │
│  │  id (PK)        │◄───────│  user_id (FK)   │                │
│  │  email          │        │  id (PK)        │                │
│  │  ...            │        │  title          │                │
│  └─────────────────┘        │  status         │                │
│                             │  ...            │                │
│                             └─────────────────┘                │
│                                                                 │
└────────────────────────────────────────────────────────────────┘
```

---

## State Transitions

### Session Lifecycle

```
[No Session] ──(first message)──► [Active]
                                      │
                                      │ (message received)
                                      ▼
                                  [Active] ◄───────────────────┐
                                      │                        │
                                      │ (30 min inactivity)    │
                                      ▼                        │
                                 [Expired] ──(new message)────►│
                                      │
                                      │ (cleanup)
                                      ▼
                                 [Deleted]
```

### Confirmation State Machine

```
[None] ──(destructive action detected)──► [Pending]
                                              │
                           ┌──────────────────┼──────────────────┐
                           │                  │                  │
                           │ (user: "yes")    │ (user: "no")     │ (5 min timeout)
                           ▼                  ▼                  ▼
                      [Executed]          [Cancelled]        [Expired]
                           │                  │                  │
                           └──────────────────┴──────────────────┘
                                              │
                                              ▼
                                           [None]
```

---

## Validation Rules

### ChatMessage

| Field | Rule |
|-------|------|
| role | Must be "user" or "assistant" |
| content | 1-2000 characters |
| timestamp | Must be valid datetime |

### ChatSession

| Field | Rule |
|-------|------|
| session_id | Valid UUID v4 |
| user_id | Must exist in users table |
| last_activity | Must be within TTL (30 min) |

### SessionContext

| Field | Rule |
|-------|------|
| messages | Max 20 entries (FIFO) |
| last_task_ids | Max 10 entries (FIFO) |
| last_intent | One of: read, create, update, delete, complete, plan, chat |

### Confirmation

| Field | Rule |
|-------|------|
| action | Must be "delete" or "bulk_update" |
| target_ids | At least 1 UUID |
| expires_at | Max 5 minutes from creation |

---

## No Database Migrations Required

Phase 3 introduces **in-memory session storage only**. No database schema changes are needed because:

1. Sessions are ephemeral (30-min TTL)
2. Sessions reference existing users/tasks tables via UUID
3. No conversation history persistence (deferred to Phase 4)

**Phase 4 Migration Preview** (not in scope):
```sql
-- Future: Conversation history table
CREATE TABLE conversations (
    id UUID PRIMARY KEY,
    user_id UUID REFERENCES users(id),
    started_at TIMESTAMPTZ,
    ended_at TIMESTAMPTZ
);

CREATE TABLE messages (
    id UUID PRIMARY KEY,
    conversation_id UUID REFERENCES conversations(id),
    role VARCHAR(10),
    content TEXT,
    metadata JSONB,
    created_at TIMESTAMPTZ
);
```

---

## Indexes (Future)

For Phase 4 database-backed sessions:

| Table | Column(s) | Type | Purpose |
|-------|-----------|------|---------|
| conversations | user_id | BTREE | User's conversation list |
| conversations | started_at | BTREE | Recent conversations |
| messages | conversation_id | BTREE | Messages in conversation |
| messages | created_at | BTREE | Message ordering |
