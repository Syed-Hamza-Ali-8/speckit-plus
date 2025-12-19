# ADR-004: Real-time Communication Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-19
- **Feature:** Phase 3 - Todo AI Chatbot
- **Context:** The AI chatbot needs to deliver responses to users. LLM responses can take 2-10 seconds; users expect feedback during processing. We must decide between synchronous responses (full response at once) vs. streaming (incremental tokens), and the transport protocol (REST, WebSocket, SSE).

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? ✅ Yes - defines client-server communication
     2) Alternatives: Multiple viable options considered with tradeoffs? ✅ Yes - REST, WebSocket, SSE, gRPC
     3) Scope: Cross-cutting concern (not an isolated detail)? ✅ Yes - affects frontend, backend, UX
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

We adopt a **dual-transport strategy** with REST for simple requests and WebSocket for streaming:

| Component | Technology | Purpose |
|-----------|------------|---------|
| **Sync Endpoint** | `POST /chat` (REST) | Simple queries, mobile clients, fallback |
| **Stream Endpoint** | `WS /chat/stream` | Real-time token streaming, tool events |
| **Auth** | JWT in header (REST) / query param (WS) | Unified authentication |
| **Serialization** | JSON | Universal client support |

### Endpoint Specifications

**REST Endpoint (Synchronous)**
```
POST /chat
Authorization: Bearer <jwt>
{ "message": string, "session_id": UUID | null }

Response (after full processing):
{ "response": string, "session_id": UUID, "intent": string, "actions": [], "metadata": {} }
```

**WebSocket Endpoint (Streaming)**
```
WS /chat/stream?token=<jwt>

Client → Server: { "type": "message", "content": string, "session_id": UUID | null }

Server → Client (streaming events):
{ "type": "token", "content": "partial text" }
{ "type": "tool_start", "tool": "list_tasks", "input": {} }
{ "type": "tool_end", "tool": "list_tasks", "result": {} }
{ "type": "complete", "response": "full text", "metadata": {} }
{ "type": "error", "code": "TOOL_TIMEOUT", "message": "..." }
```

### Event Types

| Event | Purpose | When Emitted |
|-------|---------|--------------|
| `token` | Incremental response text | During LLM generation |
| `tool_start` | Tool invocation beginning | Before MCP tool executes |
| `tool_end` | Tool invocation complete | After MCP tool returns |
| `complete` | Full response ready | After all processing |
| `error` | Error occurred | On any failure |
| `pong` | Heartbeat response | Every 30s (keep-alive) |

### Client Strategy

1. **Primary**: Connect via WebSocket for streaming UX
2. **Fallback**: Use REST if WebSocket unavailable (corporate proxies, mobile)
3. **Heartbeat**: Ping every 30s to detect stale connections

## Consequences

### Positive

- **Responsive UX**: Users see tokens as they're generated (perceived latency ~200ms vs 3-5s)
- **Tool transparency**: Users see when tools are invoked, building trust
- **Graceful degradation**: REST fallback ensures universal compatibility
- **Bidirectional**: WebSocket allows server-initiated events (future: notifications)
- **Connection efficiency**: Single persistent connection vs. repeated HTTP requests

### Negative

- **Implementation complexity**: Two code paths (REST + WebSocket) to maintain
- **WebSocket state**: Must handle reconnection, heartbeats, connection pooling
- **Proxy issues**: Some corporate environments block WebSocket; REST fallback essential
- **Testing complexity**: WebSocket tests require async/await patterns
- **Resource usage**: Persistent connections consume server memory

## Alternatives Considered

### Alternative A: REST-Only (No Streaming)

- **Approach**: Single `POST /chat` endpoint that waits for full response
- **Pros**: Simplest implementation, universal compatibility, stateless
- **Cons**: Poor UX (3-10s wait with no feedback), no tool visibility
- **Why rejected**: Unacceptable perceived latency for conversational AI

### Alternative B: Server-Sent Events (SSE)

- **Approach**: `GET /chat/stream` with SSE for server-to-client streaming
- **Pros**: Simpler than WebSocket, auto-reconnect, HTTP/2 multiplexing
- **Cons**: Unidirectional (client can't send mid-stream), less browser control
- **Why rejected**: Need bidirectional for session management and future features

### Alternative C: gRPC Streaming

- **Approach**: Bidirectional gRPC stream with Protocol Buffers
- **Pros**: Efficient binary protocol, strong typing, built-in streaming
- **Cons**: Browser support requires gRPC-Web proxy, complex setup, learning curve
- **Why rejected**: Overkill for our use case; REST+WebSocket is more accessible

### Alternative D: Long Polling

- **Approach**: Client polls `/chat/status` until response ready
- **Pros**: Works everywhere, simple implementation
- **Cons**: Inefficient (many requests), higher latency, complex client logic
- **Why rejected**: WebSocket provides better efficiency and UX

## References

- Feature Spec: `specs/phase3-ai-chatbot/spec.md` (Sections 1.1, 1.2)
- Architecture Diagram: `specs/phase3-ai-chatbot/architecture.md` (WebSocket flow)
- Frontend Integration: `phase2/frontend/src/services/chatApi.ts` (to be created)
- Related ADRs: ADR-002 (agent chain provides events for tool_start/tool_end)
