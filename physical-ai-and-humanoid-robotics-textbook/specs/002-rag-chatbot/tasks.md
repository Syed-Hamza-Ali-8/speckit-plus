# Tasks: RAG Chatbot

**Input**: Design documents from `/specs/002-rag-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Not explicitly requested, so only core implementation tasks will be generated.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `backend/tests/`
- Paths shown below assume this structure.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for the RAG chatbot backend.

- [ ] T001 Create backend directory structure (`backend/src/`, `backend/tests/`, etc.)
- [ ] T002 Initialize Python environment in `backend/` with `requirements.txt`
- [ ] T003 Install core dependencies: `fastapi`, `uvicorn`, `qdrant-client`, `psycopg2-binary`, `openai`, `python-dotenv`, `pypdf`, `python-multipart`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete.

- [ ] T004 Create base FastAPI application instance in `backend/src/main.py`
- [ ] T005 Configure environment variable loading (`.env`) in `backend/src/config.py`
- [ ] T006 Implement basic logging configuration in `backend/src/logger.py`
- [ ] T007 Setup database connection for Neon Postgres in `backend/src/database/postgres.py`
- [ ] T008 Setup Qdrant client connection in `backend/src/database/qdrant.py`
- [ ] T009 Define common utility functions in `backend/src/utils/common.py`
- [ ] T010 Implement API key authentication middleware in `backend/src/middleware/auth.py`
- [ ] T011 Create base `router` in `backend/src/api/v1/router.py`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel.

---

## Phase 3: User Story 1 - Document Ingestion (Priority: P1) 🎯 MVP

**Goal**: Enable uploading, processing, chunking, embedding, and storing documents in Qdrant and Neon Postgres.

**Independent Test**: Successfully upload a PDF/TXT/MD file via the `/ingest_document` endpoint, verify its metadata in Postgres, and check for embeddings in Qdrant.

### Implementation for User Story 1

- [ ] T012 [P] [US1] Create `Document` Pydantic model in `backend/src/models/document.py` (for request/response)
- [ ] T013 [P] [US1] Create `DocumentMetadata` SQL model for Neon Postgres in `backend/src/models/metadata.py`
- [ ] T014 [US1] Implement document parsing utility (`PDF, TXT, MD`) in `backend/src/utils/document_parser.py`
- [ ] T015 [US1] Implement text chunking logic in `backend/src/services/chunking_service.py`
- [ ] T016 [US1] Implement embedding generation using OpenAI in `backend/src/services/embedding_service.py`
- [ ] T017 [US1] Implement Qdrant interaction service (collection creation, point insertion) in `backend/src/services/qdrant_service.py`
- [ ] T018 [US1] Implement Neon Postgres metadata service (CRUD for `DocumentMetadata`) in `backend/src/services/postgres_service.py`
- [ ] T019 [US1] Create the `/ingest_document` API endpoint in `backend/src/api/v1/ingestion.py`
- [ ] T020 [US1] Integrate document parser, chunking, embedding, Qdrant, and Postgres services in `backend/src/api/v1/ingestion.py`
- [ ] T021 [US1] Add input validation and error handling for `/ingest_document`
- [ ] T022 [US1] Add logging for document ingestion operations

**Checkpoint**: User Story 1 should be fully functional and testable independently.

---

## Phase 4: User Story 2 - RAG Querying (Priority: P1)

**Goal**: Enable natural language querying against ingested documents to get grounded answers.

**Independent Test**: Successfully query the `/rag_query` endpoint with a question and receive a grounded answer with source references.

### Implementation for User Story 2

- [ ] T023 [P] [US2] Create `QueryRequest` and `QueryResponse` Pydantic models in `backend/src/models/query.py`
- [ ] T024 [US2] Implement document retrieval logic from Qdrant in `backend/src/services/retrieval_service.py` (depends on `qdrant_service.py`)
- [ ] T025 [US2] Implement RAG response generation using OpenAI ChatKit in `backend/src/services/rag_service.py`
- [ ] T026 [US2] Create the `/rag_query` API endpoint in `backend/src/api/v1/query.py`
- [ ] T027 [US2] Integrate retrieval and RAG services in `backend/src/api/v1/query.py`
- [ ] T028 [US2] Add input validation and error handling for `/rag_query`
- [ ] T029 [US2] Add logging for RAG querying operations

**Checkpoint**: User Stories 1 AND 2 should both work independently.

---

## Phase 5: User Story 3 - Health Check (Priority: P2)

**Goal**: Provide an endpoint to monitor the operational status of the service and its dependencies.

**Independent Test**: Access the `/health` endpoint and verify a 200 OK status with dependency statuses.

### Implementation for User Story 3

- [ ] T030 [US3] Implement health check logic for Neon Postgres in `backend/src/services/health_service.py`
- [ ] T031 [US3] Implement health check logic for Qdrant in `backend/src/services/health_service.py`
- [ ] T032 [US3] Create the `/health` API endpoint in `backend/src/api/v1/health.py`
- [ ] T033 [US3] Integrate health check services into `/health` endpoint
- [ ] T034 [US3] Add logging for health check operations

**Checkpoint**: All user stories should now be independently functional.

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories.

- [ ] T035 Create a `Dockerfile` for the FastAPI backend in `backend/Dockerfile`
- [ ] T036 Create a `docker-compose.yml` for local development (FastAPI, Qdrant, Postgres) in `docker-compose.yml`
- [ ] T037 Write deployment instructions (e.g., Kubernetes, serverless) in `docs/deployment.md`
- [ ] T038 Review API documentation (e.g., OpenAPI spec) and refine it
- [ ] T039 Implement comprehensive error handling strategy across all endpoints
- [ ] T040 Implement unit tests for core services (e.g., chunking, embedding, Qdrant, Postgres services) in `backend/tests/unit/`
- [ ] T041 Implement integration tests for API endpoints (ingestion, query, health) in `backend/tests/integration/`
- [ ] T042 Conduct basic performance testing on `/rag_query` endpoint

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1 - Document Ingestion)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1 - RAG Querying)**: Can start after Foundational (Phase 2) - Depends on US1 for ingested data, but implementation can proceed in parallel up to integration.
- **User Story 3 (P2 - Health Check)**: Can start after Foundational (Phase 2) - No dependencies on other stories.

### Within Each User Story

- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (implicitly handled by file paths)
- Once Foundational phase completes, User Stories 1, 2 (partial), and 3 can start in parallel (if team capacity allows)
- Within User Story 1, tasks T012, T013 are [P].
- Within User Story 2, task T023 is [P].
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all models for User Story 1 together:
Task: "Create Document Pydantic model in backend/src/models/document.py"
Task: "Create DocumentMetadata SQL model for Neon Postgres in backend/src/models/metadata.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 + Foundational)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Document Ingestion)
   - Developer B: User Story 2 (RAG Querying)
   - Developer C: User Story 3 (Health Check)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
