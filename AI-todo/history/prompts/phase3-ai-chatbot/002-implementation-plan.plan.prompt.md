---
id: 002
title: Phase 3 Implementation Plan - 5 Part Roadmap
stage: plan
date: 2025-12-19
surface: agent
model: claude-sonnet-4.5
feature: phase3-ai-chatbot
branch: main
user: hamza
command: /sp.plan
labels: [plan, implementation, roadmap, agents, mcp, chat-ui]
links:
  spec: specs/phase3-ai-chatbot/spec.md
  ticket: null
  adr: history/adr/ADR-002-multi-agent-orchestration-architecture.md
  pr: null
files:
  - specs/phase3-ai-chatbot/research.md
  - specs/phase3-ai-chatbot/data-model.md
  - specs/phase3-ai-chatbot/contracts/chat-api.yaml
  - specs/phase3-ai-chatbot/quickstart.md
  - specs/phase3-ai-chatbot/plan.md
tests:
  - N/A (planning phase)
---

## Prompt

/sp.plan

## Phase 3: Todo AI Chatbot
From specs/phase3-ai-chatbot/spec.md

Generate 5-part implementation roadmap:
1. Phase 3.1: MCP + OpenAI Infrastructure (20m)
2. Phase 3.2: Chatkit UI Production (25m)
3. Phase 3.3: Core Task Subagents (20m)
4. Phase 3.4: Conversation + Safety (15m)
5. Phase 3.5: Session Memory + Tests (10m)

Output format:
- 5 phases with time estimates
- Dependencies between phases
- File changes per phase
- Success criteria per phase

## Response snapshot

Generated comprehensive 5-part implementation plan (90 min total):

**Phase 3.1: MCP + OpenAI Infrastructure (20 min)**
- 7 files: config, MCP schemas, tools, base agent
- Dependencies: Phase 2 backend
- Success: TaskTools and BaseAgent importable

**Phase 3.2: Chatkit UI Production (25 min)**
- 11 files: ChatPage, components, hooks, API
- Dependencies: Phase 2 frontend (parallel with 3.1)
- Success: /chat renders with glassmorphism

**Phase 3.3: Core Task Subagents (20 min)**
- 2 files: TaskManagerAgent, PlannerAgent
- Dependencies: Phase 3.1
- Success: Agents handle CRUD and planning

**Phase 3.4: Conversation + Safety (15 min)**
- 3 files: ConversationAgent, GuardrailAgent, router
- Dependencies: Phase 3.3
- Success: Intent classification, confirmation flow

**Phase 3.5: Session Memory + Tests (10 min)**
- 6 files: Session store, chat endpoint, tests
- Dependencies: Phase 3.4
- Success: POST /chat works, >80% coverage

**Supporting Artifacts Created:**
- research.md: 8 technical decisions resolved
- data-model.md: Session entities defined
- contracts/chat-api.yaml: OpenAPI spec
- quickstart.md: Developer setup guide

## Outcome

- ✅ Impact: Complete implementation roadmap with 29 files across 5 phases
- 🧪 Tests: Test plan defined in Phase 3.5
- 📁 Files: 5 planning artifacts created
- 🔁 Next prompts: /sp.tasks to generate detailed task breakdown
- 🧠 Reflection: User-requested time estimates (20/25/20/15/10 min) were reasonable and adopted

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL):
  - Constitution compliance: PASS (Phase III tech, clean architecture)
  - Dependency ordering: PASS (3.1→3.3→3.4→3.5, 3.2 parallel)
  - File coverage: PASS (all files from spec.md covered)
  - Time estimates: PASS (total 90 min reasonable for scope)
- Prompt variant (if applicable): Standard /sp.plan with user constraints
- Next experiment (smallest change to try): Add parallel execution markers between 3.1 and 3.2
