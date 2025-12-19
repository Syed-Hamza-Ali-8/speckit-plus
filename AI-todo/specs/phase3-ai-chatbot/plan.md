# Implementation Plan — Phase 3: Todo AI Chatbot

## Overview

**Total Duration**: 90 minutes (5 phases)
**Branch**: `feature/phase3-ai-chatbot`
**Dependencies**: Phase 2 backend (FastAPI + SQLModel) + frontend (React + Vite)

---

## Constitution Compliance Check

| Rule | Status | Notes |
|------|--------|-------|
| Spec-Driven Development | ✅ | spec.md, research.md, data-model.md created |
| Phase III Technology | ✅ | MCP tools, OpenAI SDK per constitution |
| Clean Architecture | ✅ | Agents layer separate from services |
| Type Hints | ✅ | All Python code fully typed |
| 80% Coverage | ⏳ | Phase 3.5 adds tests |
| JWT Auth | ✅ | Reuses Phase 2 auth |

---

## Dependency Graph

```
Phase 3.1: MCP + OpenAI Infrastructure
    │
    ├──────────────────────────────────────┐
    │                                      │
    ▼                                      ▼
Phase 3.2: Chatkit UI              Phase 3.3: Core Task Subagents
    │                                      │
    │                                      │
    │                                      ▼
    │                              Phase 3.4: Conversation + Safety
    │                                      │
    └──────────────────────────────────────┤
                                           │
                                           ▼
                                   Phase 3.5: Session Memory + Tests
```

---

## Phase 3.1: MCP + OpenAI Infrastructure (20 min)

### Objective
Establish foundational infrastructure for AI chat: OpenAI client, MCP tool schemas, and base agent class.

### Dependencies
- Phase 2 backend running
- OpenAI API key configured

### File Changes

| File | Action | Description |
|------|--------|-------------|
| `app/core/config.py` | MODIFY | Add OpenAI/session config settings |
| `app/mcp/__init__.py` | CREATE | MCP module exports |
| `app/mcp/schemas.py` | CREATE | Tool input/output Pydantic schemas |
| `app/mcp/tools.py` | CREATE | 5 MCP tool implementations |
| `app/agents/__init__.py` | CREATE | Agent module exports |
| `app/agents/base.py` | CREATE | BaseAgent abstract class |
| `pyproject.toml` | MODIFY | Add openai, websockets deps |

### Implementation Details

#### 3.1.1: Update Config (2 min)

```python
# app/core/config.py - additions
class Settings(BaseSettings):
    # OpenAI
    openai_api_key: str = ""
    openai_model: str = "gpt-4o-mini"
    openai_timeout: int = 30

    # Session
    session_ttl_minutes: int = 30
    session_max_messages: int = 20

    # Chat
    chat_rate_limit_per_minute: int = 60
```

#### 3.1.2: MCP Schemas (5 min)

```python
# app/mcp/schemas.py
class MCPToolResult(BaseModel):
    success: bool
    data: dict | list | None = None
    error: str | None = None
    error_code: str | None = None

class ListTasksInput(BaseModel):
    status: Literal["pending", "completed"] | None = None
    limit: int = Field(default=20, ge=1, le=100)
    offset: int = Field(default=0, ge=0)

class CreateTaskInput(BaseModel):
    title: str = Field(..., min_length=1, max_length=200)
    description: str | None = Field(default=None, max_length=1000)
    due_date: date | None = None

# ... update_task, complete_task, delete_task inputs
```

#### 3.1.3: MCP Tools (8 min)

```python
# app/mcp/tools.py
class TaskTools:
    @staticmethod
    async def list_tasks(db: AsyncSession, user_id: UUID, input: ListTasksInput) -> MCPToolResult:
        tasks, total = await task_service.get_tasks(db, user_id, ...)
        return MCPToolResult(success=True, data={"tasks": [...], "total": total})

    @staticmethod
    async def create_task(db: AsyncSession, user_id: UUID, input: CreateTaskInput) -> MCPToolResult:
        task = await task_service.create_task(db, user_id, TaskCreate(...))
        return MCPToolResult(success=True, data=task.model_dump())

    # ... update_task, complete_task, delete_task
```

#### 3.1.4: Base Agent (5 min)

```python
# app/agents/base.py
class AgentRequest(BaseModel):
    intent: str
    user_id: UUID
    session_id: UUID
    message: str
    context: SessionContext | None = None

class AgentResponse(BaseModel):
    success: bool
    data: dict | None = None
    error: str | None = None
    actions_taken: list[ActionTaken] = []
    requires_confirmation: bool = False
    confirmation_prompt: str | None = None

class BaseAgent(ABC):
    name: str

    @abstractmethod
    async def process(self, request: AgentRequest, db: AsyncSession) -> AgentResponse:
        pass
```

### Success Criteria
- [ ] `from app.mcp import TaskTools` imports successfully
- [ ] `from app.agents import BaseAgent` imports successfully
- [ ] Config loads OpenAI settings from env
- [ ] All 5 tool schemas validate correctly

---

## Phase 3.2: Chatkit UI Production (25 min)

### Objective
Build custom chat UI with glassmorphism design, message rendering, and input handling.

### Dependencies
- Phase 2 frontend running
- Phase 3.1 NOT required (UI-only)

### File Changes

| File | Action | Description |
|------|--------|-------------|
| `src/pages/ChatPage.tsx` | CREATE | Main chat page |
| `src/components/chat/ChatContainer.tsx` | CREATE | Glassmorphism wrapper |
| `src/components/chat/MessageList.tsx` | CREATE | Message history display |
| `src/components/chat/MessageBubble.tsx` | CREATE | Individual message |
| `src/components/chat/InputBar.tsx` | CREATE | Text input + send |
| `src/components/chat/TypingIndicator.tsx` | CREATE | Loading animation |
| `src/components/chat/ActionChip.tsx` | CREATE | Tool action badge |
| `src/components/chat/TaskCard.tsx` | CREATE | Inline task preview |
| `src/hooks/useChat.ts` | CREATE | Chat state hook |
| `src/services/chatApi.ts` | CREATE | API integration |
| `src/App.tsx` | MODIFY | Add /chat route |

### Implementation Details

#### 3.2.1: Chat Page & Container (5 min)

```tsx
// src/pages/ChatPage.tsx
export default function ChatPage() {
  const { messages, sendMessage, isLoading } = useChat();

  return (
    <ChatContainer>
      <MessageList messages={messages} />
      <InputBar onSend={sendMessage} disabled={isLoading} />
    </ChatContainer>
  );
}

// src/components/chat/ChatContainer.tsx
export function ChatContainer({ children }) {
  return (
    <div className="h-screen flex flex-col bg-gradient-to-br from-purple-900/20 to-blue-900/20">
      <div className="flex-1 backdrop-blur-lg bg-white/10 dark:bg-black/20 rounded-2xl m-4 shadow-xl border border-white/20">
        {children}
      </div>
    </div>
  );
}
```

#### 3.2.2: Message Components (8 min)

```tsx
// src/components/chat/MessageBubble.tsx
export function MessageBubble({ message }) {
  const isUser = message.role === 'user';

  return (
    <div className={`flex ${isUser ? 'justify-end' : 'justify-start'}`}>
      <div className={`max-w-[80%] rounded-2xl px-4 py-2 ${
        isUser
          ? 'bg-blue-500 text-white'
          : 'bg-white/10 backdrop-blur-sm text-white'
      }`}>
        {message.content}
        {message.actions?.map(action => (
          <ActionChip key={action.tool} action={action} />
        ))}
      </div>
    </div>
  );
}
```

#### 3.2.3: Input Bar (5 min)

```tsx
// src/components/chat/InputBar.tsx
export function InputBar({ onSend, disabled }) {
  const [message, setMessage] = useState('');

  const handleSubmit = (e) => {
    e.preventDefault();
    if (message.trim() && !disabled) {
      onSend(message);
      setMessage('');
    }
  };

  return (
    <form onSubmit={handleSubmit} className="p-4 border-t border-white/10">
      <div className="flex gap-2">
        <input
          type="text"
          value={message}
          onChange={(e) => setMessage(e.target.value)}
          placeholder="Ask me about your tasks..."
          disabled={disabled}
          className="flex-1 bg-white/10 rounded-full px-4 py-2 text-white placeholder-white/50"
        />
        <button
          type="submit"
          disabled={disabled || !message.trim()}
          className="bg-blue-500 rounded-full px-4 py-2 text-white disabled:opacity-50"
        >
          Send
        </button>
      </div>
    </form>
  );
}
```

#### 3.2.4: useChat Hook + API (7 min)

```tsx
// src/hooks/useChat.ts
export function useChat() {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);

  const sendMessage = async (content: string) => {
    setIsLoading(true);
    setMessages(prev => [...prev, { role: 'user', content }]);

    try {
      const response = await chatApi.send(content, sessionId);
      setSessionId(response.session_id);
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: response.response,
        actions: response.actions
      }]);
    } catch (error) {
      // Handle error
    } finally {
      setIsLoading(false);
    }
  };

  return { messages, sendMessage, isLoading };
}
```

### Success Criteria
- [ ] `/chat` route renders chat UI
- [ ] Messages display user (right) and assistant (left)
- [ ] Glassmorphism styling applied
- [ ] Input bar works (send on Enter)
- [ ] Dark mode supported
- [ ] Mobile responsive (320px+)

---

## Phase 3.3: Core Task Subagents (20 min)

### Objective
Implement TaskManagerAgent and PlannerAgent with MCP tool integration.

### Dependencies
- Phase 3.1 complete (MCP tools, base agent)

### File Changes

| File | Action | Description |
|------|--------|-------------|
| `app/agents/task_manager.py` | CREATE | TaskManagerAgent implementation |
| `app/agents/planner.py` | CREATE | PlannerAgent implementation |

### Implementation Details

#### 3.3.1: TaskManagerAgent (12 min)

```python
# app/agents/task_manager.py
class TaskManagerAgent(BaseAgent):
    name = "TaskManagerAgent"

    async def process(self, request: AgentRequest, db: AsyncSession) -> AgentResponse:
        actions = []

        if request.intent == "read":
            result = await TaskTools.list_tasks(
                db, request.user_id,
                ListTasksInput()
            )
            actions.append(ActionTaken(tool="list_tasks", success=result.success, summary="..."))
            return AgentResponse(success=True, data=result.data, actions_taken=actions)

        elif request.intent == "create":
            # Extract task details from message using OpenAI
            task_data = await self._extract_task_data(request.message)
            result = await TaskTools.create_task(db, request.user_id, task_data)
            actions.append(ActionTaken(tool="create_task", ...))
            return AgentResponse(success=True, data=result.data, actions_taken=actions)

        elif request.intent == "delete":
            # Require confirmation for destructive actions
            return AgentResponse(
                success=True,
                requires_confirmation=True,
                confirmation_prompt=f"⚠️ Are you sure you want to delete this task?"
            )

        # ... update, complete handlers
```

#### 3.3.2: PlannerAgent (8 min)

```python
# app/agents/planner.py
class PlannerAgent(BaseAgent):
    name = "PlannerAgent"

    async def process(self, request: AgentRequest, db: AsyncSession) -> AgentResponse:
        # Fetch all tasks for planning
        result = await TaskTools.list_tasks(
            db, request.user_id,
            ListTasksInput(limit=100)
        )

        if not result.success:
            return AgentResponse(success=False, error=result.error)

        tasks = result.data["tasks"]

        # Use OpenAI to generate plan
        plan = await self._generate_plan(tasks, request.message)

        return AgentResponse(
            success=True,
            data={"plan": plan},
            actions_taken=[ActionTaken(tool="list_tasks", success=True, summary="Analyzed tasks")]
        )

    async def _generate_plan(self, tasks: list, query: str) -> str:
        # Call OpenAI to generate daily/weekly plan
        response = await openai_client.chat.completions.create(
            model=settings.openai_model,
            messages=[
                {"role": "system", "content": PLANNER_SYSTEM_PROMPT},
                {"role": "user", "content": f"Tasks: {json.dumps(tasks)}\n\nRequest: {query}"}
            ]
        )
        return response.choices[0].message.content
```

### Success Criteria
- [ ] TaskManagerAgent handles read/create/update/delete/complete
- [ ] PlannerAgent generates daily/weekly plans
- [ ] Both agents return proper AgentResponse
- [ ] Delete operations trigger confirmation requirement

---

## Phase 3.4: Conversation + Safety (15 min)

### Objective
Implement ConversationAgent (intent classification, routing) and GuardrailAgent (safety checks).

### Dependencies
- Phase 3.3 complete (task agents)

### File Changes

| File | Action | Description |
|------|--------|-------------|
| `app/agents/conversation.py` | CREATE | ConversationAgent + intent classification |
| `app/agents/guardrails.py` | CREATE | GuardrailAgent + safety checks |
| `app/agents/router.py` | CREATE | Multi-agent router |

### Implementation Details

#### 3.4.1: ConversationAgent (8 min)

```python
# app/agents/conversation.py
INTENT_FUNCTIONS = [{
    "name": "classify_intent",
    "parameters": {
        "type": "object",
        "properties": {
            "intent": {"enum": ["read", "create", "update", "delete", "complete", "plan", "chat"]},
            "task_id": {"type": "string"},
            "task_title": {"type": "string"},
            "task_description": {"type": "string"}
        },
        "required": ["intent"]
    }
}]

class ConversationAgent(BaseAgent):
    name = "ConversationAgent"

    async def classify_intent(self, message: str) -> dict:
        response = await openai_client.chat.completions.create(
            model=settings.openai_model,
            messages=[
                {"role": "system", "content": "Classify the user's intent for a todo app."},
                {"role": "user", "content": message}
            ],
            functions=INTENT_FUNCTIONS,
            function_call={"name": "classify_intent"}
        )
        return json.loads(response.choices[0].message.function_call.arguments)

    async def process(self, request: AgentRequest, db: AsyncSession) -> AgentResponse:
        # Classify intent
        classification = await self.classify_intent(request.message)
        intent = classification["intent"]

        # Update request with classified intent
        request.intent = intent

        # Route to appropriate agent
        if intent == "chat":
            return self._handle_chat(request.message)
        elif intent == "plan":
            return await planner_agent.process(request, db)
        else:
            # Task operations go through guardrails first
            return await guardrail_agent.validate_and_execute(request, db)
```

#### 3.4.2: GuardrailAgent (5 min)

```python
# app/agents/guardrails.py
class GuardrailAgent(BaseAgent):
    name = "GuardrailAgent"

    DESTRUCTIVE_INTENTS = {"delete"}

    async def validate_and_execute(self, request: AgentRequest, db: AsyncSession) -> AgentResponse:
        # Pre-execution validation
        if request.intent in self.DESTRUCTIVE_INTENTS:
            # Check if confirmation is pending
            if request.context and request.context.pending_confirmation:
                if request.message.lower() in ["yes", "y", "do it", "confirm"]:
                    # Execute with confirmation
                    return await task_manager_agent.process(request, db)
                else:
                    return AgentResponse(success=True, data={"message": "Deletion cancelled."})

            # Request confirmation
            return AgentResponse(
                success=True,
                requires_confirmation=True,
                confirmation_prompt="⚠️ Are you sure? Reply 'yes' to confirm."
            )

        # Non-destructive operations proceed directly
        return await task_manager_agent.process(request, db)
```

#### 3.4.3: Agent Router (2 min)

```python
# app/agents/router.py
class AgentRouter:
    def __init__(self):
        self.conversation = ConversationAgent()
        self.task_manager = TaskManagerAgent()
        self.planner = PlannerAgent()
        self.guardrails = GuardrailAgent()

    async def route(self, request: AgentRequest, db: AsyncSession) -> AgentResponse:
        return await self.conversation.process(request, db)
```

### Success Criteria
- [ ] ConversationAgent classifies intents correctly
- [ ] GuardrailAgent blocks destructive actions without confirmation
- [ ] Agent router orchestrates full flow
- [ ] Intent classification >90% accuracy on test cases

---

## Phase 3.5: Session Memory + Tests (10 min)

### Objective
Implement in-memory session storage, chat endpoints, and integration tests.

### Dependencies
- Phase 3.4 complete (all agents)

### File Changes

| File | Action | Description |
|------|--------|-------------|
| `app/chat/__init__.py` | CREATE | Chat module exports |
| `app/chat/session.py` | CREATE | InMemorySessionStore |
| `app/chat/schemas.py` | CREATE | ChatRequest/Response schemas |
| `app/chat/router.py` | CREATE | /chat endpoint |
| `app/chat/websocket.py` | CREATE | WebSocket handler |
| `app/main.py` | MODIFY | Mount chat router |
| `tests/test_chat/` | CREATE | Integration tests |

### Implementation Details

#### 3.5.1: Session Memory (3 min)

```python
# app/chat/session.py
class InMemorySessionStore:
    _sessions: dict[UUID, ChatSession] = {}

    def get(self, session_id: UUID) -> ChatSession | None:
        session = self._sessions.get(session_id)
        if session and self._is_expired(session):
            del self._sessions[session_id]
            return None
        return session

    def create(self, user_id: UUID) -> ChatSession:
        session = ChatSession(
            session_id=uuid4(),
            user_id=user_id,
            created_at=datetime.utcnow(),
            last_activity=datetime.utcnow(),
            context=SessionContext()
        )
        self._sessions[session.session_id] = session
        return session

    def update(self, session: ChatSession) -> None:
        session.last_activity = datetime.utcnow()
        self._sessions[session.session_id] = session
```

#### 3.5.2: Chat Endpoint (4 min)

```python
# app/chat/router.py
router = APIRouter(prefix="/chat", tags=["chat"])

@router.post("", response_model=ChatResponse)
@limiter.limit("60/minute")
async def chat(
    request: ChatRequest,
    user: CurrentUser,
    db: DbSession,
):
    start_time = time.time()

    # Get or create session
    session = session_store.get(request.session_id) if request.session_id else None
    if not session:
        session = session_store.create(user.id)

    # Build agent request
    agent_request = AgentRequest(
        intent="",  # Will be classified
        user_id=user.id,
        session_id=session.session_id,
        message=request.message,
        context=session.context
    )

    # Process through agent router
    response = await agent_router.route(agent_request, db)

    # Update session
    session.context.messages.append(ChatMessage(role="user", content=request.message))
    session.context.messages.append(ChatMessage(role="assistant", content=response.data.get("message", "")))
    session_store.update(session)

    return ChatResponse(
        response=response.data.get("message", ""),
        session_id=session.session_id,
        intent=agent_request.intent,
        actions=response.actions_taken,
        metadata=ChatMetadata(
            processing_time_ms=int((time.time() - start_time) * 1000),
            agent_chain=[...],
            model=settings.openai_model
        )
    )
```

#### 3.5.3: Integration Tests (3 min)

```python
# tests/test_chat/test_chat_endpoint.py
@pytest.mark.asyncio
async def test_create_task_via_chat(client, auth_token):
    response = await client.post(
        "/chat",
        headers={"Authorization": f"Bearer {auth_token}"},
        json={"message": "Add a task to buy groceries"}
    )
    assert response.status_code == 200
    data = response.json()
    assert data["intent"] == "create"
    assert any(a["tool"] == "create_task" for a in data["actions"])

@pytest.mark.asyncio
async def test_delete_requires_confirmation(client, auth_token):
    response = await client.post(
        "/chat",
        headers={"Authorization": f"Bearer {auth_token}"},
        json={"message": "Delete task 1"}
    )
    assert response.status_code == 200
    assert "confirm" in response.json()["response"].lower()
```

### Success Criteria
- [ ] POST /chat returns valid response
- [ ] Session persists across requests
- [ ] Context resolution works ("that task")
- [ ] All tests pass
- [ ] Coverage >80% for chat module

---

## Summary

| Phase | Duration | Files | Dependencies |
|-------|----------|-------|--------------|
| 3.1: MCP + OpenAI | 20 min | 7 | Phase 2 |
| 3.2: Chatkit UI | 25 min | 11 | Phase 2 frontend |
| 3.3: Task Subagents | 20 min | 2 | Phase 3.1 |
| 3.4: Conversation + Safety | 15 min | 3 | Phase 3.3 |
| 3.5: Session + Tests | 10 min | 6 | Phase 3.4 |
| **Total** | **90 min** | **29** | - |

---

## Related ADRs

- ADR-002: Multi-Agent Orchestration Architecture
- ADR-003: MCP Tool Layer for Task Operations
- ADR-004: Real-time Communication Strategy
- ADR-005: Session Memory Architecture
