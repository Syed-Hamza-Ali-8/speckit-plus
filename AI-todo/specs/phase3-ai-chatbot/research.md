# Research — Phase 3: Todo AI Chatbot

## Technical Decisions Resolved

### 1. OpenAI SDK Integration Pattern

**Decision**: Use `openai>=1.0.0` Python SDK with streaming support

**Rationale**:
- Native async/await support aligns with FastAPI
- Built-in streaming via `client.chat.completions.create(stream=True)`
- Function calling for tool invocation
- Type-safe responses with Pydantic models

**Alternatives Considered**:
- LangChain: Rejected (heavy dependency, unnecessary abstraction)
- Direct HTTP calls: Rejected (no retry logic, no streaming helpers)
- Anthropic SDK: Rejected (spec requires OpenAI compatibility)

**Implementation Pattern**:
```python
from openai import AsyncOpenAI

client = AsyncOpenAI(api_key=settings.openai_api_key)

async def chat_completion(messages: list[dict], tools: list[dict] = None):
    response = await client.chat.completions.create(
        model=settings.openai_model,
        messages=messages,
        tools=tools,
        stream=True,
    )
    async for chunk in response:
        yield chunk.choices[0].delta.content
```

---

### 2. MCP Server Implementation

**Decision**: Custom MCP-like tool layer (MCP SDK not yet stable for Python)

**Rationale**:
- Official MCP SDK is TypeScript-first; Python support is experimental
- Implement MCP-compatible patterns without SDK dependency
- Tools follow MCP schema conventions for future migration

**Alternatives Considered**:
- Wait for stable MCP Python SDK: Rejected (timeline uncertain)
- Use TypeScript MCP SDK: Rejected (adds Node.js dependency)
- Use LangChain tools: Rejected (different abstraction)

**Implementation Pattern**:
```python
class MCPTool(ABC):
    name: str
    description: str
    input_schema: type[BaseModel]

    @abstractmethod
    async def execute(self, input: BaseModel, context: ToolContext) -> MCPToolResult:
        pass
```

---

### 3. WebSocket Streaming Architecture

**Decision**: FastAPI native WebSocket with JSON message protocol

**Rationale**:
- FastAPI has built-in WebSocket support
- JSON serialization is universal, debuggable
- Matches existing frontend expectations (React)

**Alternatives Considered**:
- Socket.IO: Rejected (additional dependency, overkill)
- Server-Sent Events: Rejected (unidirectional, no session support)
- gRPC streaming: Rejected (requires protobuf, complex setup)

**Implementation Pattern**:
```python
@router.websocket("/chat/stream")
async def websocket_chat(websocket: WebSocket, token: str = Query(...)):
    user = await validate_jwt(token)
    await websocket.accept()

    async for event in process_message(message, user):
        await websocket.send_json(event.dict())
```

---

### 4. Intent Classification Approach

**Decision**: LLM-based classification using OpenAI function calling

**Rationale**:
- Single LLM call handles both classification and extraction
- No separate NLU model needed
- Function calling provides structured output

**Alternatives Considered**:
- Rule-based regex: Rejected (inflexible, poor accuracy)
- Separate classifier model: Rejected (adds latency, complexity)
- Fine-tuned model: Rejected (maintenance overhead)

**Implementation Pattern**:
```python
INTENT_SCHEMA = {
    "name": "classify_intent",
    "parameters": {
        "type": "object",
        "properties": {
            "intent": {"enum": ["read", "create", "update", "delete", "complete", "plan", "chat"]},
            "entities": {"type": "object"}
        }
    }
}
```

---

### 5. Session Memory Storage

**Decision**: In-memory dict with TTL-based cleanup

**Rationale**:
- Simplest implementation for MVP
- No external dependencies (Redis)
- Clear upgrade path to Redis in Phase 4

**Alternatives Considered**:
- Redis from start: Rejected (infrastructure overhead for MVP)
- Database sessions: Rejected (wrong tool for ephemeral data)
- JWT-encoded sessions: Rejected (token size explosion)

**Implementation Pattern**:
```python
class InMemorySessionStore:
    _sessions: dict[UUID, ChatSession] = {}
    _ttl_seconds: int = 1800  # 30 minutes

    def get(self, session_id: UUID) -> ChatSession | None:
        session = self._sessions.get(session_id)
        if session and self._is_expired(session):
            del self._sessions[session_id]
            return None
        return session
```

---

### 6. Chatkit vs Custom Chat UI

**Decision**: Build custom chat UI components (Chatkit doesn't exist as specified)

**Rationale**:
- No `@chatkit/react` package exists in npm registry
- Custom components allow full control over glassmorphism styling
- Can integrate WebSocket directly without adapter

**Alternatives Considered**:
- stream-chat-react: Rejected (requires Stream.io account, overkill)
- @chatscope/chat-ui-kit-react: Considered but has limited styling flexibility
- Custom from scratch: Selected (best fit for requirements)

**Implementation Pattern**:
```typescript
// Custom chat components
<ChatContainer>
  <MessageList messages={messages} />
  <InputBar onSend={handleSend} disabled={isLoading} />
</ChatContainer>
```

---

### 7. Rate Limiting Strategy

**Decision**: Reuse existing slowapi middleware with chat-specific limits

**Rationale**:
- Phase 2 already has slowapi integrated
- Can add route-specific decorators for chat endpoints
- Per-user tracking already implemented

**Implementation Pattern**:
```python
@router.post("/chat")
@limiter.limit("60/minute", key_func=get_user_id_from_request)
async def chat(request: ChatRequest, user: CurrentUser):
    pass
```

---

### 8. Error Handling Strategy

**Decision**: Structured error responses with codes for client handling

**Rationale**:
- Clients can programmatically handle specific error types
- User-friendly messages separate from error codes
- Consistent with Phase 2 patterns

**Error Code Mapping**:
| Code | HTTP | User Message |
|------|------|--------------|
| AUTH_REQUIRED | 401 | "Please log in to continue" |
| RATE_LIMITED | 429 | "Too many requests. Please wait a moment." |
| INTENT_UNCLEAR | 400 | "I didn't understand that. Can you rephrase?" |
| TOOL_TIMEOUT | 504 | "The operation took too long. Please try again." |
| CONFIRMATION_REQUIRED | 200 | (prompt in response body) |

---

## Dependencies Verification

### Backend Dependencies

| Package | Version | Purpose | Status |
|---------|---------|---------|--------|
| `openai` | >=1.0.0 | OpenAI API client | ✅ Available |
| `websockets` | >=12.0 | WebSocket support | ✅ Available |
| `pydantic` | >=2.0.0 | Schema validation | ✅ Already installed |

### Frontend Dependencies

| Package | Version | Purpose | Status |
|---------|---------|---------|--------|
| `@chatkit/react` | N/A | Chat UI kit | ❌ Does not exist |
| Custom components | N/A | Chat UI | ✅ Build from scratch |

---

## Best Practices Applied

### 1. Agent Design

- **Single Responsibility**: Each agent handles one concern
- **Explicit Contracts**: AgentRequest/AgentResponse schemas
- **Stateless Agents**: All state in session, not agents
- **Guardrails First**: Safety checks before any mutation

### 2. WebSocket Design

- **Heartbeat**: 30s ping to detect stale connections
- **Reconnection**: Client handles reconnect with session restoration
- **Graceful Shutdown**: Server sends close frame before disconnect

### 3. OpenAI Integration

- **Streaming**: Always stream for UX responsiveness
- **Timeout**: 30s max per API call
- **Retry**: 3 retries with exponential backoff
- **Cost Optimization**: Use gpt-4o-mini for chat (90% cheaper than gpt-4)

---

## Risk Mitigations

| Risk | Mitigation |
|------|------------|
| OpenAI rate limits | Use organization API key with higher limits |
| Token costs | gpt-4o-mini + message length limits |
| Session memory pressure | 30-min TTL + 20 message cap |
| WebSocket disconnects | Client-side reconnect logic |
| Intent misclassification | Fallback to "unclear" with reprompt |

---

## Resolved Unknowns

| Unknown | Resolution |
|---------|------------|
| MCP SDK availability | Build MCP-compatible layer without SDK |
| Chatkit package | Build custom chat UI components |
| Intent classification model | Use OpenAI function calling |
| Session storage backend | In-memory dict (Redis in Phase 4) |
| WebSocket auth | JWT in query param (WS standard) |
