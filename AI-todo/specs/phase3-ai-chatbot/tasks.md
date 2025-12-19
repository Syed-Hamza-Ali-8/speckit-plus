# Tasks — Phase 3: Todo AI Chatbot

## Overview

**Feature**: AI-powered chatbot for natural language task management
**Total Tasks**: 25
**Estimated Duration**: 90 minutes
**Branch**: `feature/phase3-ai-chatbot`

---

## Phase 3.1: MCP + OpenAI Infrastructure (6 tasks)

**Goal**: Establish foundational AI infrastructure with OpenAI client, MCP tool schemas, and base agent patterns.

**Independent Test Criteria**:
- `from app.mcp import TaskTools` imports successfully
- `from app.agents import BaseAgent` imports successfully
- Config loads OpenAI settings from environment
- All 5 tool schemas validate correctly

### Tasks

- [X] T001 Update pyproject.toml to add openai>=1.0.0 and websockets>=12.0 dependencies in `phase2/backend/pyproject.toml`

- [X] T002 Add OpenAI and session config settings to `phase2/backend/app/core/config.py` (openai_api_key, openai_model, openai_timeout, session_ttl_minutes, session_max_messages, chat_rate_limit_per_minute)

- [X] T003 [P] Create MCP module with schemas in `phase2/backend/app/mcp/__init__.py` and `phase2/backend/app/mcp/schemas.py` (MCPToolResult, ListTasksInput, CreateTaskInput, UpdateTaskInput, CompleteTaskInput, DeleteTaskInput)

- [X] T004 Implement 5 MCP tools (list_tasks, create_task, update_task, complete_task, delete_task) in `phase2/backend/app/mcp/tools.py`

- [X] T005 [P] Create agents module with base classes in `phase2/backend/app/agents/__init__.py` and `phase2/backend/app/agents/base.py` (AgentRequest, AgentResponse, ActionTaken, BaseAgent ABC)

- [X] T006 Create OpenAI client helper with async streaming support in `phase2/backend/app/agents/openai_client.py`

---

## Phase 3.2: Chatkit UI Production (8 tasks)

**Goal**: Build custom chat UI with glassmorphism design, message rendering, and input handling.

**Independent Test Criteria**:
- `/chat` route renders chat UI
- Messages display user (right) and assistant (left)
- Glassmorphism styling applied
- Input bar works (send on Enter)
- Dark mode supported
- Mobile responsive (320px+)

### Tasks

- [X] T007 [P] Create ChatContainer component with glassmorphism styling in `phase2/frontend/src/components/chat/ChatContainer.tsx`

- [X] T008 [P] Create MessageBubble component with user/assistant styling in `phase2/frontend/src/components/chat/MessageBubble.tsx`

- [X] T009 [P] Create MessageList component with auto-scroll in `phase2/frontend/src/components/chat/MessageList.tsx`

- [X] T010 [P] Create InputBar component with send button in `phase2/frontend/src/components/chat/InputBar.tsx`

- [X] T011 [P] Create TypingIndicator component (three-dot animation) in `phase2/frontend/src/components/chat/TypingIndicator.tsx`

- [X] T012 [P] Create ActionChip component for tool action badges in `phase2/frontend/src/components/chat/ActionChip.tsx`

- [X] T013 Create chatApi service and useChat hook in `phase2/frontend/src/services/chatApi.ts` and `phase2/frontend/src/hooks/useChat.ts`

- [X] T014 Create ChatPage and add /chat route in `phase2/frontend/src/pages/ChatPage.tsx` and update `phase2/frontend/src/App.tsx`

---

## Phase 3.3: Core Task Subagents (6 tasks)

**Goal**: Implement TaskManagerAgent and PlannerAgent with MCP tool integration.

**Dependencies**: Phase 3.1 complete

**Independent Test Criteria**:
- TaskManagerAgent handles read/create/update/delete/complete intents
- PlannerAgent generates daily/weekly plans
- Both agents return proper AgentResponse
- Delete operations trigger confirmation requirement

### Tasks

- [X] T015 Implement TaskManagerAgent with read intent handler in `phase2/backend/app/agents/task_manager.py`

- [X] T016 Add create/update/complete intent handlers to TaskManagerAgent in `phase2/backend/app/agents/task_manager.py`

- [X] T017 Add delete intent handler with confirmation requirement to TaskManagerAgent in `phase2/backend/app/agents/task_manager.py`

- [X] T018 Implement PlannerAgent with daily plan generation in `phase2/backend/app/agents/planner.py`

- [X] T019 Add weekly plan and priority matrix generation to PlannerAgent in `phase2/backend/app/agents/planner.py`

- [X] T020 Create OpenAI function schemas for entity extraction (task title, description, due_date) in `phase2/backend/app/agents/schemas.py`

---

## Phase 3.4: Conversation + Safety (3 tasks)

**Goal**: Implement ConversationAgent (intent classification, routing) and GuardrailAgent (safety checks).

**Dependencies**: Phase 3.3 complete

**Independent Test Criteria**:
- ConversationAgent classifies intents correctly (>90% accuracy)
- GuardrailAgent blocks destructive actions without confirmation
- Agent router orchestrates full flow
- Confirmation flow works for delete operations

### Tasks

- [X] T021 Implement ConversationAgent with intent classification using OpenAI function calling in `phase2/backend/app/agents/conversation.py`

- [X] T022 Implement GuardrailAgent with destructive action detection and confirmation flow in `phase2/backend/app/agents/guardrails.py`

- [X] T023 Create AgentRouter to orchestrate multi-agent flow in `phase2/backend/app/agents/router.py`

---

## Phase 3.5: Session Memory + Tests (2 tasks)

**Goal**: Implement in-memory session storage, chat endpoints, and integration tests.

**Dependencies**: Phase 3.4 complete

**Independent Test Criteria**:
- POST /chat returns valid response
- Session persists across requests
- Context resolution works ("that task")
- WebSocket streams tokens
- All tests pass with >80% coverage

### Tasks

- [X] T024 Create chat module with session store, schemas, and POST /chat endpoint in `phase2/backend/app/chat/__init__.py`, `phase2/backend/app/chat/session.py`, `phase2/backend/app/chat/schemas.py`, `phase2/backend/app/chat/router.py` and mount in `phase2/backend/app/main.py`

- [X] T025 Create WebSocket handler and integration tests in `phase2/backend/app/chat/websocket.py` and `phase2/backend/tests/test_chat/test_chat_endpoint.py`

---

## Dependency Graph

```
Phase 3.1: MCP + OpenAI Infrastructure
    T001 → T002 → T003 → T004 → T006
                    │
                    └──→ T005
    │
    ├──────────────────────────────────────┐
    │                                      │
    ▼                                      ▼
Phase 3.2: Chatkit UI              Phase 3.3: Core Task Subagents
(T007-T014) [PARALLEL]              T015 → T016 → T017
                                    T018 → T019
                                         │
                                         └──→ T020
    │                                      │
    │                                      ▼
    │                              Phase 3.4: Conversation + Safety
    │                               T021 → T022 → T023
    │                                      │
    └──────────────────────────────────────┤
                                           │
                                           ▼
                                   Phase 3.5: Session + Tests
                                    T024 → T025
```

---

## Parallel Execution Opportunities

### Phase 3.1 (within phase)
- T003 (MCP schemas) || T005 (Agent base classes) - different modules

### Phase 3.2 (entire phase parallelizable)
- T007 || T008 || T009 || T010 || T011 || T012 - independent components
- T013 depends on component completion
- T014 depends on T013

### Cross-Phase Parallelism
- Phase 3.2 (frontend) || Phase 3.3 (backend agents) - can run in parallel after 3.1

---

## File Summary

### Backend (17 files)

| Phase | File | Action |
|-------|------|--------|
| 3.1 | `pyproject.toml` | MODIFY |
| 3.1 | `app/core/config.py` | MODIFY |
| 3.1 | `app/mcp/__init__.py` | CREATE |
| 3.1 | `app/mcp/schemas.py` | CREATE |
| 3.1 | `app/mcp/tools.py` | CREATE |
| 3.1 | `app/agents/__init__.py` | CREATE |
| 3.1 | `app/agents/base.py` | CREATE |
| 3.1 | `app/agents/openai_client.py` | CREATE |
| 3.3 | `app/agents/task_manager.py` | CREATE |
| 3.3 | `app/agents/planner.py` | CREATE |
| 3.3 | `app/agents/schemas.py` | CREATE |
| 3.4 | `app/agents/conversation.py` | CREATE |
| 3.4 | `app/agents/guardrails.py` | CREATE |
| 3.4 | `app/agents/router.py` | CREATE |
| 3.5 | `app/chat/__init__.py` | CREATE |
| 3.5 | `app/chat/session.py` | CREATE |
| 3.5 | `app/chat/schemas.py` | CREATE |
| 3.5 | `app/chat/router.py` | CREATE |
| 3.5 | `app/chat/websocket.py` | CREATE |
| 3.5 | `app/main.py` | MODIFY |
| 3.5 | `tests/test_chat/test_chat_endpoint.py` | CREATE |

### Frontend (10 files)

| Phase | File | Action |
|-------|------|--------|
| 3.2 | `src/components/chat/ChatContainer.tsx` | CREATE |
| 3.2 | `src/components/chat/MessageBubble.tsx` | CREATE |
| 3.2 | `src/components/chat/MessageList.tsx` | CREATE |
| 3.2 | `src/components/chat/InputBar.tsx` | CREATE |
| 3.2 | `src/components/chat/TypingIndicator.tsx` | CREATE |
| 3.2 | `src/components/chat/ActionChip.tsx` | CREATE |
| 3.2 | `src/services/chatApi.ts` | CREATE |
| 3.2 | `src/hooks/useChat.ts` | CREATE |
| 3.2 | `src/pages/ChatPage.tsx` | CREATE |
| 3.2 | `src/App.tsx` | MODIFY |

---

## Implementation Strategy

### MVP Scope (Phase 3.1 + 3.5 minimal)
1. Complete Phase 3.1 (MCP + OpenAI infrastructure)
2. Implement basic POST /chat endpoint
3. Test with simple read/create intents

### Incremental Delivery
1. **Milestone 1**: Infrastructure (T001-T006) - Backend can process messages
2. **Milestone 2**: UI Shell (T007-T014) - Frontend can send/receive messages
3. **Milestone 3**: Agent Intelligence (T015-T023) - Full intent handling
4. **Milestone 4**: Production Ready (T024-T025) - Sessions, WebSocket, tests

---

## Task Count Summary

| Phase | Tasks | Parallelizable |
|-------|-------|----------------|
| 3.1: MCP + OpenAI | 6 | 2 |
| 3.2: Chatkit UI | 8 | 6 |
| 3.3: Core Subagents | 6 | 0 |
| 3.4: Conversation + Safety | 3 | 0 |
| 3.5: Session + Tests | 2 | 0 |
| **Total** | **25** | **8** |

---

## Related Documents

- Specification: `specs/phase3-ai-chatbot/spec.md`
- Implementation Plan: `specs/phase3-ai-chatbot/plan.md`
- Data Model: `specs/phase3-ai-chatbot/data-model.md`
- API Contract: `specs/phase3-ai-chatbot/contracts/chat-api.yaml`
- Research: `specs/phase3-ai-chatbot/research.md`
- ADRs: ADR-002, ADR-003, ADR-004, ADR-005
