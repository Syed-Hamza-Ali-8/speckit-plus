# Quickstart — Phase 3: Todo AI Chatbot

## Prerequisites

- Phase 2 backend running (`uvicorn app.main:app`)
- Phase 2 frontend running (`npm run dev`)
- OpenAI API key
- Node.js 18+, Python 3.12+

## Environment Setup

### 1. Add Environment Variables

Add to `phase2/backend/.env`:

```env
# OpenAI Configuration
OPENAI_API_KEY=sk-your-api-key-here
OPENAI_MODEL=gpt-4o-mini
OPENAI_TIMEOUT=30

# Session Configuration
SESSION_TTL_MINUTES=30
SESSION_MAX_MESSAGES=20

# Chat Rate Limiting
CHAT_RATE_LIMIT_PER_MINUTE=60
```

### 2. Install Backend Dependencies

```bash
cd phase2/backend
uv add openai websockets
```

### 3. Install Frontend Dependencies

```bash
cd phase2/frontend
npm install
# No additional packages needed (custom chat UI)
```

## Quick Verification

### 1. Start Backend

```bash
cd phase2/backend
uvicorn app.main:app --reload
```

### 2. Test Chat Endpoint

```bash
# Get JWT token first (from Phase 2 auth)
TOKEN=$(curl -s -X POST http://localhost:8000/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"password123"}' \
  | jq -r '.access_token')

# Test chat endpoint
curl -X POST http://localhost:8000/chat \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"message": "Show my tasks"}'
```

Expected response:
```json
{
  "response": "You have 3 tasks...",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "intent": "read",
  "actions": [{"tool": "list_tasks", "success": true, "summary": "Listed 3 tasks"}],
  "metadata": {"processing_time_ms": 1234, "agent_chain": ["ConversationAgent", "TaskManagerAgent"], "model": "gpt-4o-mini"}
}
```

### 3. Test WebSocket Streaming

```bash
# Using websocat (install: cargo install websocat)
websocat "ws://localhost:8000/chat/stream?token=$TOKEN"

# Send message
{"type": "message", "content": "Add a task to test websocket", "session_id": null}
```

### 4. Start Frontend

```bash
cd phase2/frontend
npm run dev
```

Navigate to `http://localhost:5173/chat`

## Implementation Order

```
Phase 3.1 → Phase 3.2 → Phase 3.3 → Phase 3.4 → Phase 3.5
    │           │           │           │           │
    │           │           │           │           └── Tests
    │           │           │           └── Conversation + Safety
    │           │           └── Task Subagents
    │           └── Chat UI
    └── MCP + OpenAI Infrastructure
```

## File Changes Summary

| Phase | Backend Files | Frontend Files |
|-------|---------------|----------------|
| 3.1 | 6 new | 0 |
| 3.2 | 0 | 11 new |
| 3.3 | 2 new | 0 |
| 3.4 | 3 new | 0 |
| 3.5 | 1 modified | 0, tests only |

## Troubleshooting

### OpenAI API Error

```
Error: OPENAI_ERROR - Invalid API key
```

**Fix**: Verify `OPENAI_API_KEY` in `.env` is valid.

### Rate Limited

```
Error: RATE_LIMITED - Too many requests
```

**Fix**: Wait 1 minute or increase `CHAT_RATE_LIMIT_PER_MINUTE`.

### Session Expired

```
Error: SESSION_EXPIRED - Session no longer valid
```

**Fix**: Start a new conversation (omit `session_id`).

### WebSocket Connection Failed

```
Error: WebSocket connection refused
```

**Fix**: Ensure backend is running and token is valid.

## Success Checklist

- [ ] `POST /chat` returns valid response
- [ ] WebSocket streams tokens
- [ ] Tasks can be created via chat
- [ ] Tasks can be listed via chat
- [ ] Destructive actions require confirmation
- [ ] Chat UI renders on `/chat`
- [ ] Dark mode works
- [ ] Mobile responsive
