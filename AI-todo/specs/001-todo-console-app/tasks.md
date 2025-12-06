---
description: "Task list for Todo Console Application implementation"
---

# Tasks: Todo Console Application

**Input**: Design documents from `/specs/001-todo-console-app/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests included as specified in the feature requirements.
**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create project structure per implementation plan
- [x] T002 [P] Create src directory structure: src/, src/models/, src/services/, src/cli/
- [x] T003 [P] Create tests directory structure: tests/unit/, tests/integration/, tests/contract/
- [x] T004 Create basic README.md with setup instructions
- [x] T005 Initialize Python project files (pyproject.toml or requirements.txt)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T006 Create Task model in src/models/task.py with id, title, description, completed fields
- [x] T007 Create TaskManager class in src/models/task.py for in-memory storage
- [x] T008 [P] Create basic CLI argument parser in src/main.py
- [x] T009 [P] Create TaskService in src/services/task_service.py with basic operations
- [x] T010 Set up error handling and validation for task operations

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Add New Tasks (Priority: P1) 🎯 MVP

**Goal**: Enable users to create new todo tasks with a title and description

**Independent Test**: Can be fully tested by adding various tasks with different titles and descriptions and verifying they appear in the task list.

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [x] T011 [P] [US1] Unit test for Task creation in tests/unit/test_task.py
- [x] T012 [P] [US1] Unit test for TaskService add_task method in tests/unit/test_task_service.py
- [x] T013 [P] [US1] Integration test for add task CLI command in tests/integration/test_cli.py

### Implementation for User Story 1

- [x] T014 [P] [US1] Implement Task creation with validation in src/models/task.py
- [x] T015 [P] [US1] Implement add_task method in TaskService in src/services/task_service.py
- [x] T016 [US1] Implement CLI add command in src/cli/cli.py
- [x] T017 [US1] Connect CLI add command to TaskService in src/main.py
- [x] T018 [US1] Add validation for title (non-empty) in src/services/task_service.py

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - View All Tasks (Priority: P1)

**Goal**: Enable users to see all their current tasks with status indicators

**Independent Test**: Can be fully tested by adding tasks and then viewing the complete list with proper status indicators.

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [x] T019 [P] [US2] Unit test for TaskService list_tasks method in tests/unit/test_task_service.py
- [x] T020 [P] [US2] Integration test for view tasks CLI command in tests/integration/test_cli.py

### Implementation for User Story 2

- [x] T021 [P] [US2] Implement list_tasks method in TaskService in src/services/task_service.py
- [x] T022 [US2] Implement CLI view/list command in src/cli/cli.py
- [x] T023 [US2] Connect CLI view command to TaskService in src/main.py
- [x] T024 [US2] Format task display with ID, title, description, and completion status in src/cli/cli.py

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Mark Tasks Complete/Incomplete (Priority: P1)

**Goal**: Enable users to update the status of their tasks as they complete them

**Independent Test**: Can be fully tested by marking tasks as complete/incomplete and verifying the status changes persist in the view.

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [x] T025 [P] [US3] Unit test for TaskService mark_complete/mark_incomplete methods in tests/unit/test_task_service.py
- [x] T026 [P] [US3] Integration test for mark complete/incomplete CLI commands in tests/integration/test_cli.py

### Implementation for User Story 3

- [x] T027 [P] [US3] Implement mark_complete and mark_incomplete methods in TaskService in src/services/task_service.py
- [x] T028 [US3] Implement CLI complete/incomplete commands in src/cli/cli.py
- [x] T029 [US3] Connect CLI complete/incomplete commands to TaskService in src/main.py
- [x] T030 [US3] Add validation for task ID existence in src/services/task_service.py

**Checkpoint**: At this point, User Stories 1, 2 AND 3 should all work independently

---

## Phase 6: User Story 4 - Update Task Details (Priority: P2)

**Goal**: Enable users to modify the details of an existing task

**Independent Test**: Can be fully tested by updating task details and verifying the changes appear when viewing tasks.

### Tests for User Story 4 (OPTIONAL - only if tests requested) ⚠️

- [x] T031 [P] [US4] Unit test for TaskService update_task method in tests/unit/test_task_service.py
- [x] T032 [P] [US4] Integration test for update task CLI command in tests/integration/test_cli.py

### Implementation for User Story 4

- [x] T033 [P] [US4] Implement update_task method in TaskService in src/services/task_service.py
- [x] T034 [US4] Implement CLI update command in src/cli/cli.py
- [x] T035 [US4] Connect CLI update command to TaskService in src/main.py
- [x] T036 [US4] Add validation for task ID existence and title in src/services/task_service.py

**Checkpoint**: At this point, User Stories 1, 2, 3 AND 4 should all work independently

---

## Phase 7: User Story 5 - Delete Tasks (Priority: P2)

**Goal**: Enable users to remove tasks that are no longer relevant or needed

**Independent Test**: Can be fully tested by deleting tasks and verifying they no longer appear in the task list.

### Tests for User Story 5 (OPTIONAL - only if tests requested) ⚠️

- [x] T037 [P] [US5] Unit test for TaskService delete_task method in tests/unit/test_task_service.py
- [x] T038 [P] [US5] Integration test for delete task CLI command in tests/integration/test_cli.py

### Implementation for User Story 5

- [x] T039 [P] [US5] Implement delete_task method in TaskService in src/services/task_service.py
- [x] T040 [US5] Implement CLI delete command in src/cli/cli.py
- [x] T041 [US5] Connect CLI delete command to TaskService in src/main.py
- [x] T042 [US5] Add validation for task ID existence in src/services/task_service.py

**Checkpoint**: All user stories should now be independently functional

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T043 [P] Add error handling for invalid task IDs across all operations in src/services/task_service.py
- [x] T044 [P] Add input validation for all CLI commands in src/cli/cli.py
- [x] T045 [P] Add help messages and usage instructions to CLI in src/cli/cli.py
- [x] T046 [P] Add logging for all operations in src/services/task_service.py
- [x] T047 [P] Add edge case handling (empty title, very long inputs, special characters) in src/services/task_service.py
- [x] T048 [P] Documentation updates in README.md with usage instructions
- [x] T049 [P] Add comprehensive tests in tests/unit/ and tests/integration/
- [x] T050 Run quickstart validation with all commands

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

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 5 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
Task: "Unit test for Task creation in tests/unit/test_task.py"
Task: "Unit test for TaskService add_task method in tests/unit/test_task_service.py"
Task: "Integration test for add task CLI command in tests/integration/test_cli.py"

# Launch all implementation tasks for User Story 1 together:
Task: "Implement Task creation with validation in src/models/task.py"
Task: "Implement add_task method in TaskService in src/services/task_service.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

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
5. Add User Story 4 → Test independently → Deploy/Demo
6. Add User Story 5 → Test independently → Deploy/Demo
7. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
   - Developer D: User Story 4
   - Developer E: User Story 5
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence