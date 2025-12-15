# Tasks: Dynamic Profile Page

**Input**: Design documents from `/specs/dynamic-profile-page/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/profile-api.yaml

**Tests**: Not explicitly requested - tests are optional and omitted for speed.

**Organization**: Tasks organized for ~2 min implementation each, 1 file per task.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: US1=Backend Profile API, US2=Frontend Types/API, US3=Profile UI
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `phase2/backend/app/`
- **Frontend**: `phase2/frontend/src/`

---

## Phase 1: Setup

**Purpose**: No setup required - extending existing project structure

*Skip - project already initialized with FastAPI + React + RTK Query*

---

## Phase 2: Foundational (Backend API)

**Purpose**: Backend PATCH /auth/me endpoint must exist before frontend can use it

**⚠️ CRITICAL**: Frontend tasks depend on this phase completing first

- [X] T001 [US1] Add `UserUpdate` schema in `phase2/backend/app/schemas/auth.py`
  - Fields: `first_name`, `last_name`, `avatar_url` (all optional, max 100 chars for names)
  - Add `avatar_url` to existing `UserResponse` schema

- [X] T002 [US1] Add `update_user_profile()` function in `phase2/backend/app/services/auth_service.py`
  - Accepts User and UserUpdate, applies partial update, returns updated User
  - Use `model_dump(exclude_unset=True)` pattern

- [X] T003 [US1] Add `PATCH /auth/me` endpoint in `phase2/backend/app/api/routes/auth.py`
  - Use `CurrentUser` dependency, call `update_user_profile` service
  - Return `UserResponse`

**Checkpoint**: Backend ready - `curl -X PATCH /api/auth/me` should work

---

## Phase 3: User Story 2 - Frontend Types & API (Priority: P1)

**Goal**: RTK Query hooks for fetching and updating user profile

**Independent Test**: TypeScript compiles, hooks are exported

- [X] T004 [P] [US2] Create user types in `phase2/frontend/src/types/user.ts`
  - `User` interface matching `UserResponse`
  - `UserUpdateRequest` interface
  - `TaskStats` interface

- [X] T005 [P] [US2] Create Zod validation in `phase2/frontend/src/lib/validations/profile.ts`
  - `profileSchema` with firstName/lastName max 100 chars
  - Export `ProfileFormData` type

- [X] T006 [US2] Create RTK Query endpoints in `phase2/frontend/src/services/userApi.ts`
  - `useGetCurrentUserQuery()` → GET /auth/me, providesTags: ['User']
  - `useUpdateProfileMutation()` → PATCH /auth/me, invalidatesTags: ['User']

**Checkpoint**: `import { useGetCurrentUserQuery, useUpdateProfileMutation } from '@/services/userApi'` works

---

## Phase 4: User Story 3 - Profile UI (Priority: P2)

**Goal**: Profile page with header, stats, and edit form

**Independent Test**: Navigate to `/profile` and see user data

- [X] T007 [US3] Add PROFILE route in `phase2/frontend/src/routes/index.tsx` and wire up in `phase2/frontend/src/App.tsx`
  - Add `PROFILE: '/profile'` to ROUTES
  - Add protected route for ProfilePage

- [X] T008 [US3] Create `phase2/frontend/src/pages/ProfilePage.tsx`
  - Fetch user with `useGetCurrentUserQuery()`
  - Layout: Avatar, name, email, member since, task stats, edit button
  - Loading/error states

**Checkpoint**: Profile page visible at `/profile` with real user data

---

## Phase 5: Polish

**Purpose**: Profile editing functionality

- [X] T009 [US3] Add profile link in `phase2/frontend/src/components/layout/Header.tsx`
  - Add Profile link/button next to logout in user menu

**Checkpoint**: Full feature complete - view profile, edit name, see stats

---

## Dependencies & Execution Order

```
T001 → T002 → T003 (Backend - sequential)
         ↓
    ┌────┴────┐
    ↓         ↓
  T004      T005  (Frontend Types - parallel)
    └────┬────┘
         ↓
       T006 (API hooks)
         ↓
       T007 (Routing)
         ↓
       T008 (ProfilePage)
         ↓
       T009 (Header link)
```

### Phase Dependencies

- **Phase 2 (Backend)**: T001 → T002 → T003 (sequential)
- **Phase 3 (Types/API)**: T004, T005 parallel, then T006
- **Phase 4 (UI)**: T007 → T008 (sequential)
- **Phase 5 (Polish)**: T009 after T008

### Parallel Opportunities

```bash
# After T003 completes, launch in parallel:
T004: "Create user types in phase2/frontend/src/types/user.ts"
T005: "Create Zod validation in phase2/frontend/src/lib/validations/profile.ts"
```

---

## Implementation Strategy

### MVP First (Tasks T001-T008)

1. Complete T001-T003 (Backend) → API ready
2. Complete T004-T006 (Frontend API layer) → Hooks ready
3. Complete T007-T008 (UI) → Profile page works
4. **STOP and VALIDATE**: Can view profile at `/profile`

### Quick Implementation Order

| Task | Time | File | Description |
|------|------|------|-------------|
| T001 | 2m | schemas/auth.py | Add UserUpdate schema |
| T002 | 2m | services/auth_service.py | Add update function |
| T003 | 2m | api/routes/auth.py | Add PATCH endpoint |
| T004 | 2m | types/user.ts | User types |
| T005 | 2m | validations/profile.ts | Zod schema |
| T006 | 2m | services/userApi.ts | RTK hooks |
| T007 | 2m | routes + App.tsx | Add route |
| T008 | 3m | ProfilePage.tsx | Main component |
| T009 | 1m | Header.tsx | Profile link |

**Total**: ~18 minutes for 9 tasks

---

## Summary

| Metric | Value |
|--------|-------|
| Total Tasks | 9 |
| Backend Tasks | 3 (T001-T003) |
| Frontend Tasks | 6 (T004-T009) |
| Parallel Opportunities | T004+T005 |
| MVP Scope | T001-T008 (8 tasks) |

---

## Notes

- Each task targets exactly 1 file
- [P] tasks can run in parallel
- Backend must complete before frontend API layer
- ProfilePage combines header + stats + edit in one component for simplicity
- Avatar upload deferred (can use inline base64 input for MVP)
