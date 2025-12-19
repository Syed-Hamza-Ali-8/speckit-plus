# Specification — Phase 3: Todo AI Chatbot

## Feature Overview

Build a complete AI-powered chatbot interface that enables natural language task management. Users interact with their todos through conversational commands, with intelligent planning suggestions and real-time streaming responses.

**Surface**: Full-stack (FastAPI Backend + React Frontend)
**Duration**: 3 hours
**Success Metric**: Users can create, read, update, delete, and plan tasks entirely through natural language chat.

---

## System Architecture

### Skills (9 Total)

| # | Skill | Purpose | Owner Agent |
|---|-------|---------|-------------|
| 1 | **ReadUserTasks** | Fetch user's tasks with filters (status, due_date) | TaskManagerAgent |
| 2 | **ModifyUserTasks** | Create, update, delete tasks (with confirmation) | TaskManagerAgent |
| 3 | **PlanAndSummarizeTasks** | Generate summaries, daily/weekly plans, priorities | PlannerAgent |
| 4 | **ChatSessionMemory** | Track conversation context within session | ConversationAgent |
| 5 | **ChatbotInterfaceUI** | Chatkit configuration and endpoint wiring | ConversationAgent |
| 6 | **OpenAI-Agents-SDK-Integrator** | Agent orchestration, tool registration, Runner | All Agents |
| 7 | **AgentGuardrails** | Pre/post execution safety validation | GuardrailAgent |
| 8 | **MCP-Server-Tools** | Expose task operations as MCP tools | MCPServerAgent |
| 9 | *(implicit)* **AuthIntegration** | Reuse Phase 2 JWT auth for chat endpoint | - |

### Subagents (5 Total)

| # | Agent | Role | Color | Skills Used |
|---|-------|------|-------|-------------|
| 1 | **ConversationAgent** | Primary interface, intent classification, response generation | 🔵 Blue | ChatSessionMemory, ChatbotInterfaceUI |
| 2 | **TaskManagerAgent** | Execute task CRUD operations via tools | 🔷 Cyan | ReadUserTasks, ModifyUserTasks, AgentGuardrails |
| 3 | **PlannerAgent** | Read-only task analysis and planning | 🟡 Yellow | PlanAndSummarizeTasks |
| 4 | **GuardrailAgent** | Pre/post execution validation, safety enforcement | 🔴 Red | AgentGuardrails |
| 5 | **MCPServerAgent** | MCP tool registration and execution | 🟣 Purple | MCP-Server-Tools |

---

## Functional Specifications

### 1. Chat Endpoint (Backend)

#### 1.1 POST /chat — Synchronous Chat

```
POST /chat
Authorization: Bearer <jwt_token>
Content-Type: application/json

Request:
{
  "message": string,           // User's natural language input (1-2000 chars)
  "session_id": string | null  // Optional session for context continuity
}

Response:
{
  "response": string,          // AI-generated response text
  "session_id": string,        // Session ID (UUID)
  "intent": string,            // Detected intent (read|create|update|delete|complete|plan|chat)
  "actions": Action[],         // MCP tools invoked
  "metadata": {
    "processing_time_ms": number,
    "agent_chain": string[],   // ["ConversationAgent", "TaskManagerAgent"]
    "model": string
  }
}
```

#### 1.2 WebSocket /chat/stream — Real-time Streaming

```
WS /chat/stream?token=<jwt_token>

Client → Server:
{ "type": "message", "content": string, "session_id": string | null }

Server → Client (streaming):
{ "type": "token", "content": string }              // Incremental text
{ "type": "tool_start", "tool": string, "input": object }
{ "type": "tool_end", "tool": string, "result": object }
{ "type": "complete", "response": string, "metadata": object }
{ "type": "error", "code": string, "message": string }
```

#### 1.3 Intent Classification

| Intent | Example Messages | Routes To |
|--------|------------------|-----------|
| `read` | "Show my tasks", "What's due today?" | TaskManagerAgent → ReadUserTasks |
| `create` | "Add a task to buy groceries" | TaskManagerAgent → ModifyUserTasks |
| `update` | "Change the title of task 3" | TaskManagerAgent → ModifyUserTasks |
| `delete` | "Delete task 5" | GuardrailAgent → TaskManagerAgent |
| `complete` | "Mark task 2 as done" | TaskManagerAgent → ModifyUserTasks |
| `plan` | "Plan my day", "What should I focus on?" | PlannerAgent |
| `chat` | "Hello", "Thanks!" | ConversationAgent (direct) |

---

### 2. Multi-Agent Orchestration

#### 2.1 Request Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│                          USER MESSAGE                                │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      CONVERSATION AGENT (Blue)                       │
│  • Load session context (ChatSessionMemory)                          │
│  • Classify intent                                                   │
│  • Route to appropriate agent                                        │
└─────────────────────────────────────────────────────────────────────┘
                                    │
          ┌─────────────────────────┼─────────────────────────┐
          │                         │                         │
          ▼                         ▼                         ▼
┌─────────────────┐   ┌─────────────────────┐   ┌─────────────────┐
│ TASK MANAGER    │   │ PLANNER AGENT       │   │ DIRECT RESPONSE │
│ AGENT (Cyan)    │   │ (Yellow)            │   │                 │
│                 │   │                     │   │                 │
│ • ReadUserTasks │   │ • PlanAndSummarize  │   │ • Greeting      │
│ • ModifyUserTasks│  │ • Daily/Weekly Plan │   │ • Thanks        │
│                 │   │ • Priority Matrix   │   │ • Help          │
└─────────────────┘   └─────────────────────┘   └─────────────────┘
          │                         │
          ▼                         │
┌─────────────────┐                 │
│ GUARDRAIL AGENT │◄────────────────┘
│ (Red)           │
│                 │
│ • Pre-execution │
│ • Confirmation  │
│ • Post-validate │
└─────────────────┘
          │
          ▼
┌─────────────────┐
│ MCP SERVER      │
│ AGENT (Purple)  │
│                 │
│ • list_tasks    │
│ • create_task   │
│ • update_task   │
│ • complete_task │
│ • delete_task   │
└─────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         PHASE 2 BACKEND                              │
│                    (TaskService → Database)                          │
└─────────────────────────────────────────────────────────────────────┘
```

#### 2.2 Agent Communication Protocol

```python
# Standard agent request format
class AgentRequest:
    intent: str
    user_id: UUID
    session_id: UUID
    message: str
    context: SessionContext

# Standard agent response format
class AgentResponse:
    success: bool
    data: dict | None
    error: str | None
    actions_taken: list[Action]
    requires_confirmation: bool
    confirmation_prompt: str | None
```

---

### 3. MCP Server Tools

#### 3.1 Tool Definitions

| Tool | Description | Input Schema | Output Schema |
|------|-------------|--------------|---------------|
| `list_tasks` | Retrieve user's tasks | `{ status?, limit?, offset? }` | `{ tasks[], total, limit, offset }` |
| `create_task` | Create new task | `{ title, description?, due_date? }` | `{ task }` |
| `update_task` | Update existing task | `{ task_id, title?, description?, due_date? }` | `{ task }` |
| `complete_task` | Toggle completion | `{ task_id, completed }` | `{ task }` |
| `delete_task` | Delete task | `{ task_id, confirm }` | `{ deleted_task_id }` |

#### 3.2 MCP Tool Result Format

```python
class MCPToolResult:
    success: bool
    data: dict | list | None
    error: str | None
    error_code: str | None  # TASK_NOT_FOUND, CONFIRMATION_REQUIRED, etc.
```

---

### 4. Session Memory

#### 4.1 Session Data Structure

```python
class ChatSession:
    session_id: UUID
    user_id: UUID
    created_at: datetime
    last_activity: datetime
    context: SessionContext

class SessionContext:
    messages: list[ChatMessage]      # Last 20 messages
    last_intent: str | None
    last_task_ids: list[UUID]        # Recently referenced tasks
    pending_confirmation: Confirmation | None
```

#### 4.2 Context Resolution

| Reference | Resolution Strategy |
|-----------|---------------------|
| "that task" | `last_task_ids[0]` |
| "the one I mentioned" | Search message history |
| "yes" / "do it" | Execute `pending_confirmation` |
| "cancel" / "no" | Clear `pending_confirmation` |

#### 4.3 Session Lifecycle

- **Creation**: Auto-created on first message (no session_id)
- **Expiration**: 30 minutes of inactivity
- **Storage**: In-memory (HashMap) for Phase 3
- **Max Messages**: 20 per session context

---

### 5. Chatbot UI (Frontend)

#### 5.1 /chat Route

**URL**: `/chat`
**Auth**: Protected route (requires login)
**Layout**: Full-height chat interface with glassmorphism design

#### 5.2 Chatkit Configuration

```typescript
// Chatkit integration
<Chat
  endpoint="/api/chat"
  streamEndpoint="/api/chat/stream"
  onMessage={handleMessage}
  onError={handleError}
  placeholder="Ask me about your tasks..."
  welcomeMessage="Hi! I'm your Todo AI assistant. I can help you manage tasks, plan your day, and stay organized."
/>
```

#### 5.3 UI Components

| Component | Description |
|-----------|-------------|
| **ChatContainer** | Full-height wrapper with glassmorphism background |
| **MessageList** | Scrollable message history with auto-scroll |
| **MessageBubble** | User (right) / Assistant (left) message styling |
| **InputBar** | Text input with send button, disabled during loading |
| **TypingIndicator** | Three-dot animation during AI response |
| **ActionChip** | Visual indicator for tool invocations |
| **TaskCard** | Inline task preview in chat responses |

#### 5.4 Glassmorphism Design System

```css
/* Glassmorphism Variables */
--glass-bg: rgba(255, 255, 255, 0.1);
--glass-border: rgba(255, 255, 255, 0.2);
--glass-blur: 10px;
--glass-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);

/* Dark mode variants */
--glass-bg-dark: rgba(0, 0, 0, 0.2);
--glass-border-dark: rgba(255, 255, 255, 0.1);
```

#### 5.5 Responsive Breakpoints

| Breakpoint | Layout |
|------------|--------|
| Mobile (<640px) | Full-screen chat, bottom input bar |
| Tablet (640-1024px) | Centered chat (max-width: 600px) |
| Desktop (>1024px) | Sidebar + centered chat (max-width: 720px) |

---

### 6. Guardrail System

#### 6.1 Pre-Execution Checks

| Check | Description | Actions |
|-------|-------------|---------|
| **Auth Validation** | Verify JWT token is valid | Block if expired/invalid |
| **Rate Limiting** | 60 req/min per user | Return 429 if exceeded |
| **Input Sanitization** | Strip XSS/injection attempts | Sanitize before processing |
| **Tool Permission** | Verify agent can use requested tool | Block unauthorized tools |
| **Destructive Detection** | Identify delete/bulk operations | Require confirmation |

#### 6.2 Confirmation Flow

```
User: "Delete task 5"
    │
    ▼
GuardrailAgent: DESTRUCTIVE action detected
    │
    ▼
ConversationAgent: "⚠️ Are you sure you want to delete 'Buy groceries'?
                    This cannot be undone. Reply 'yes' to confirm."
    │
    ▼ (session stores pending_confirmation)

User: "yes"
    │
    ▼
ConversationAgent: Resolve pending_confirmation → Execute delete
    │
    ▼
Response: "✅ Task 'Buy groceries' has been deleted."
```

#### 6.3 Post-Execution Validation

| Check | Description |
|-------|-------------|
| **Result Structure** | Verify response has required fields |
| **Sensitive Data** | Ensure no PII/credentials in response |
| **State Consistency** | Verify mutations logged properly |

---

## Technical Specifications

### Backend File Structure

```
phase2/backend/app/
├── mcp/                           # MCP Server Module
│   ├── __init__.py
│   ├── server.py                  # MCP server setup
│   ├── tools.py                   # Tool implementations
│   └── schemas.py                 # Tool I/O schemas
│
├── agents/                        # Agent Orchestration
│   ├── __init__.py
│   ├── base.py                    # BaseAgent abstract class
│   ├── conversation.py            # ConversationAgent
│   ├── task_manager.py            # TaskManagerAgent
│   ├── planner.py                 # PlannerAgent
│   ├── guardrails.py              # GuardrailAgent
│   └── router.py                  # Multi-agent router
│
├── chat/                          # Chat Endpoint Module
│   ├── __init__.py
│   ├── router.py                  # FastAPI router (/chat)
│   ├── session.py                 # ChatSessionMemory
│   ├── schemas.py                 # Request/Response schemas
│   └── websocket.py               # WebSocket handler
│
└── core/
    └── config.py                  # + OpenAI/MCP config
```

### Frontend File Structure

```
phase2/frontend/src/
├── pages/
│   └── ChatPage.tsx               # /chat route
│
├── components/
│   └── chat/                      # Chat Components
│       ├── ChatContainer.tsx      # Main wrapper
│       ├── MessageList.tsx        # Message history
│       ├── MessageBubble.tsx      # Individual message
│       ├── InputBar.tsx           # Text input
│       ├── TypingIndicator.tsx    # Loading animation
│       ├── ActionChip.tsx         # Tool action display
│       └── TaskCard.tsx           # Inline task preview
│
├── services/
│   └── chatApi.ts                 # Chat API integration
│
├── hooks/
│   └── useChat.ts                 # Chat state management
│
└── stores/
    └── chatStore.ts               # Zustand chat store
```

### Dependencies

#### Backend (pyproject.toml additions)

```toml
[project.dependencies]
# Phase 3 additions
openai = ">=1.0.0"
websockets = ">=12.0"

[project.optional-dependencies]
dev = [
    "pytest-timeout>=2.0.0",
]
```

#### Frontend (package.json additions)

```json
{
  "dependencies": {
    "@chatkit/react": "^1.0.0"
  }
}
```

### Environment Variables

```env
# OpenAI Configuration
OPENAI_API_KEY=sk-...
OPENAI_MODEL=gpt-4o-mini
OPENAI_TIMEOUT=30

# MCP Configuration
MCP_TOOL_TIMEOUT=30
MCP_MAX_RETRIES=3

# Session Configuration
SESSION_TTL_MINUTES=30
SESSION_MAX_MESSAGES=20

# Chat Configuration
CHAT_RATE_LIMIT_PER_MINUTE=60
```

---

## API Contracts

### Chat Request

```python
class ChatRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=2000)
    session_id: UUID | None = None
```

### Chat Response

```python
class ChatResponse(BaseModel):
    response: str
    session_id: UUID
    intent: str
    actions: list[ActionTaken]
    metadata: ChatMetadata

class ActionTaken(BaseModel):
    tool: str
    success: bool
    summary: str

class ChatMetadata(BaseModel):
    processing_time_ms: int
    agent_chain: list[str]
    model: str
```

### WebSocket Messages

```python
class WSIncoming(BaseModel):
    type: Literal["message", "ping"]
    content: str | None = None
    session_id: UUID | None = None

class WSOutgoing(BaseModel):
    type: Literal["token", "tool_start", "tool_end", "complete", "error", "pong"]
    content: str = ""
    metadata: dict = {}
```

---

## Acceptance Criteria

### Backend

- [ ] POST /chat returns valid response within 3s (p95)
- [ ] WebSocket streams tokens in real-time (<100ms latency)
- [ ] All 5 MCP tools execute correctly
- [ ] Guardrails block destructive actions without confirmation
- [ ] Session context persists across messages
- [ ] Rate limiting enforced (60/min)
- [ ] 401 returned for invalid tokens

### Frontend

- [ ] /chat route renders Chatkit UI
- [ ] Messages display correctly (user right, AI left)
- [ ] Streaming tokens render incrementally
- [ ] Tool actions shown as chips
- [ ] Glassmorphism styling applied
- [ ] Mobile responsive (320px+)
- [ ] Dark mode supported

### Integration

- [ ] Phase 2 auth (JWT) works with chat endpoints
- [ ] Task CRUD reflects in /tasks page immediately
- [ ] Agent orchestration completes full flows
- [ ] Error states handled gracefully

---

## Error Handling

| Error Code | HTTP | Description |
|------------|------|-------------|
| `AUTH_REQUIRED` | 401 | No/invalid JWT token |
| `SESSION_EXPIRED` | 400 | Session no longer valid |
| `RATE_LIMITED` | 429 | Too many requests |
| `INTENT_UNCLEAR` | 400 | Could not classify intent |
| `TOOL_TIMEOUT` | 504 | MCP tool exceeded 30s |
| `TOOL_ERROR` | 500 | MCP tool execution failed |
| `GUARDRAIL_BLOCKED` | 403 | Operation blocked by safety check |
| `CONFIRMATION_REQUIRED` | 200 | Destructive action needs confirmation |
| `OPENAI_ERROR` | 502 | OpenAI API failure |

---

## Out of Scope

- Voice input/output (Phase 4)
- Database-backed conversation history (Phase 4)
- Custom agent creation UI
- Fine-tuned intent models
- Multi-language support
- File attachments in chat
- Task sharing via chat
- Collaborative chat sessions

---

## Security Considerations

1. **Authentication**: All chat endpoints require valid JWT
2. **Authorization**: Tasks scoped to authenticated user only
3. **Rate Limiting**: 60 requests/minute per user
4. **Input Sanitization**: Strip potential XSS/injection
5. **Confirmation**: Destructive actions require explicit consent
6. **Logging**: Audit trail for all task modifications
7. **No PII in Logs**: Sanitize user data from debug output

---

## Testing Strategy

### Unit Tests
- Intent classification accuracy (>90%)
- Tool schema validation
- Session context resolution
- Guardrail rule evaluation

### Integration Tests
- Full chat flow (message → agent → tool → response)
- WebSocket connection lifecycle
- Session persistence
- Error handling paths

### E2E Tests
- User creates task via chat
- User lists tasks via chat
- User completes task via chat
- User deletes task with confirmation
- User gets daily plan

---

## Implementation Phases

### Phase A: MCP Foundation (45 min)
1. Create `mcp/` module
2. Implement all 5 MCP tools
3. Add tool schemas with validation
4. Unit test tools

### Phase B: Agent Layer (60 min)
1. Create `agents/` module
2. Implement ConversationAgent with intent classification
3. Implement TaskManagerAgent with tool invocation
4. Implement PlannerAgent with planning logic
5. Implement GuardrailAgent with safety checks
6. Create multi-agent router

### Phase C: Chat Endpoints (30 min)
1. Create `chat/` module
2. Implement POST /chat
3. Implement WebSocket /chat/stream
4. Add session memory (in-memory)
5. Integration test full flow

### Phase D: Frontend UI (45 min)
1. Create ChatPage.tsx
2. Add Chatkit integration
3. Implement message components
4. Apply glassmorphism styling
5. Add responsive layout
6. Test dark mode

---

## Architecture Diagram Reference

See `architecture.md` for visual system diagram.
