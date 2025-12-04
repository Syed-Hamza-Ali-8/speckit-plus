# Feature Tasks: RAG Chatbot with FastAPI, Qdrant, Neon Postgres

**Feature Branch**: `001-foundation-setup`
**Created**: 2025-11-30
**Status**: Draft

This document outlines the tasks required to build a full Retrieval-Augmented Generation (RAG) chatbot with FastAPI, Qdrant, Neon Postgres, OpenAI ChatKit, and document ingestion, based on the `rag_chatbot_prompt.txt`, `plan.md`, and `spec.md`. The foundational setup (directories, Docusaurus, Python environment, skill modules, meta files) is assumed to be already completed as per the existing `tasks.md` for "01-Foundation Setup".

## Project Structure Overview

```text
backend/
├── src/
│   ├── models/           # Pydantic models for data
│   ├── services/         # Business logic, Qdrant/Postgres interactions
│   └── api/              # FastAPI endpoints
└── tests/

frontend/
├── src/
│   ├── components/       # UI components for ChatKit SDK
│   ├── pages/            # Chatbot integration page
│   └── services/         # Frontend API interaction
└── tests/

book/                     # Docusaurus project
skills/                   # Python skill modules
specs/                    # Specification documents (this feature, plan, spec, tasks)
.specify/                 # SpecKit Plus templates and scripts
CLAUDE.md                 # Project guidelines
```

---

## Phase 2: Foundational (RAG System Core)

**Goal**: Implement the core components of the RAG system, including database integrations and embedding.

- [ ] T001 Create `backend/src/models/rag_models.py` for Pydantic models (e.g., Document, Query)
- [ ] T002 Create `backend/src/services/qdrant_service.py` for Qdrant client initialization and operations
- [ ] T003 Create `backend/src/services/postgres_service.py` for Neon Postgres client initialization and operations
- [ ] T004 Implement `backend/src/services/embedding_service.py` for OpenAI `text-embedding-3-large` integration
- [ ] T005 Define Neon Postgres schema in `backend/src/sql/schema.sql` for document metadata
- [ ] T006 Implement a document ingestion pipeline in `backend/src/cli/ingest.py` to:
    - [ ] T007 Read book content (placeholder for PDF/Markdown/HTML parsing)
    - [ ] T008 Chunk content into manageable segments
    - [ ] T009 Generate embeddings using `embedding_service.py`
    - [ ] T010 Store embeddings in Qdrant via `qdrant_service.py`
    - [ ] T011 Store document metadata in Neon Postgres via `postgres_service.py`
- [ ] T012 Implement a RAG retrieval function in `backend/src/services/rag_service.py` that:
    - [ ] T013 Accepts a user question
    - [ ] T014 Generates an embedding for the question
    - [ ] T015 Queries Qdrant for similar document chunks
    - [ ] T016 Retrieves metadata from Neon Postgres for the matching chunks

---

## Phase 3: RAG Chatbot Functionality

**Goal**: Implement the FastAPI backend endpoints for the chatbot and integrate with a basic frontend UI.

- [ ] T017 Create FastAPI endpoint `/rag_query` in `backend/src/api/chatbot.py` that:
    - [ ] T018 Accepts user questions and optional selected text
    - [ ] T019 Uses `rag_service.py` for retrieval
    - [ ] T020 Generates a context-aware answer
    - [ ] T021 Returns the RAG-augmented answer
- [ ] T022 Create basic `frontend/index.html` for chatbot UI embedding
- [ ] T023 Implement `frontend/src/components/ChatbotWidget.tsx` using OpenAI ChatKit SDK
- [ ] T024 Integrate ChatbotWidget into `frontend/index.html` to connect to `/rag_query` endpoint

---

## Final Phase: Deployment & Polish

**Goal**: Provide clear deployment instructions and ensure code quality.

- [ ] T025 Document Qdrant Cloud setup instructions (API key, cluster URL)
- [ ] T026 Document Neon Serverless Postgres setup instructions (connection string, database creation)
- [ ] T027 Write FastAPI server deployment instructions (e.g., Docker, Gunicorn/Uvicorn, environment variables)
- [ ] T028 Add comments and docstrings to all newly created code for clarity
- [ ] T029 Ensure code modularity and cleanliness across `backend/` and `frontend/`
- [ ] T030 Set up basic logging for FastAPI backend in `backend/src/main.py`

---

## Dependencies

The completion order for tasks is sequential across phases. Within a phase, parallel execution opportunities are noted.

## Parallel Execution Opportunities

- In Phase 2, tasks T002 (Qdrant service) and T003 (Postgres service) can be developed in parallel.
- In Phase 2, tasks T005 (Postgres schema) and T006 (ingestion pipeline outline) can be started in parallel after database services are established.
- In Phase 3, frontend (T022-T024) and backend API (T017-T021) development can be done in parallel once the backend RAG service (T012-T016) is defined.

## Implementation Strategy

The implementation will follow an MVP (Minimum Viable Product) approach, focusing on delivering core RAG chatbot functionality incrementally. We will prioritize completing Phase 2 tasks before moving to Phase 3, ensuring a solid foundation.

---
