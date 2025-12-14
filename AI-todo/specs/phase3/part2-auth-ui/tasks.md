# Tasks: Phase 3 Part 2 - Authentication UI

**Input**: Design documents from `/specs/phase3/part2-auth-ui/`
**Prerequisites**: spec.md, plan.md
**Tests**: No tests requested for this phase (manual verification via checkpoints)

**Organization**: Tasks organized sequentially (dependencies → schemas → components → pages → redirect logic)

## Format: `[ID] [P?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- Include exact file paths in descriptions
- Checkpoints verify each phase completion

## Path Conventions

- **Frontend**: `phase2/frontend/`
- **Source**: `phase2/frontend/src/`

---

## Phase 1: Dependencies + Setup

**Purpose**: Install required packages and configure Sonner toaster

**Checkpoint**: `npm ls react-hook-form zod sonner` shows packages installed

- [X] T001 Install auth dependencies: `npm install react-hook-form @hookform/resolvers zod sonner` in `phase2/frontend/`
- [X] T002 Add Sonner `<Toaster />` component to `phase2/frontend/src/App.tsx` with proper positioning
- [X] T003 Verify TypeScript compiles: `npx tsc --noEmit` passes

**Deliverables**:
- New dependencies in package.json
- Sonner Toaster integrated in App.tsx

---

## Phase 2: Zod Validation Schemas

**Purpose**: Create type-safe validation schemas for auth forms

**Checkpoint**: Schemas export correctly, TypeScript types infer from schemas

- [X] T004 Create `phase2/frontend/src/lib/validations/auth.ts` with loginSchema (email, password)
- [X] T005 Add registerSchema with confirmPassword refinement to `phase2/frontend/src/lib/validations/auth.ts`
- [X] T006 Export TypeScript types (LoginFormData, RegisterFormData) from schemas in `phase2/frontend/src/lib/validations/auth.ts`

**Deliverables**:
- `src/lib/validations/auth.ts` with typed Zod schemas
- loginSchema: email (required, valid format), password (required, min 8)
- registerSchema: extends loginSchema + confirmPassword (must match)

---

## Phase 3: Loading Button Component

**Purpose**: Create reusable button component with loading spinner

**Checkpoint**: LoadingButton renders spinner when loading=true

- [X] T007 Create `phase2/frontend/src/components/ui/loading-button.tsx` extending shadcn Button
- [X] T008 Add Loader2 spinner icon from lucide-react to loading state in `phase2/frontend/src/components/ui/loading-button.tsx`
- [X] T009 Support `loading`, `loadingText`, and standard Button props in `phase2/frontend/src/components/ui/loading-button.tsx`

**Deliverables**:
- `LoadingButton` component with spinner
- Disabled state during loading
- Custom loading text support

---

## Phase 4: LoginForm Component

**Purpose**: Create login form with React Hook Form + Zod + RTK Query

**Checkpoint**: Login form validates, submits to API, shows toast on success/error

- [X] T010 Create `phase2/frontend/src/components/auth/LoginForm.tsx` with React Hook Form setup
- [X] T011 Integrate Zod resolver with loginSchema in `phase2/frontend/src/components/auth/LoginForm.tsx`
- [X] T012 Add email and password Input fields with FormField pattern in `phase2/frontend/src/components/auth/LoginForm.tsx`
- [X] T013 Integrate `useLoginMutation` from RTK Query in `phase2/frontend/src/components/auth/LoginForm.tsx`
- [X] T014 Add Sonner toast notifications (success: "Welcome back!", error: API message) in `phase2/frontend/src/components/auth/LoginForm.tsx`
- [X] T015 Use LoadingButton with isLoading state in `phase2/frontend/src/components/auth/LoginForm.tsx`
- [X] T016 Add inline validation error display below each field in `phase2/frontend/src/components/auth/LoginForm.tsx`

**Deliverables**:
- LoginForm component with full validation
- RTK Query integration
- Toast notifications
- Loading state handling

---

## Phase 5: RegisterForm Component

**Purpose**: Create registration form with password confirmation

**Checkpoint**: Register form validates, handles password match, submits to API

- [X] T017 Create `phase2/frontend/src/components/auth/RegisterForm.tsx` with React Hook Form setup
- [X] T018 Integrate Zod resolver with registerSchema in `phase2/frontend/src/components/auth/RegisterForm.tsx`
- [X] T019 Add email, password, confirmPassword fields in `phase2/frontend/src/components/auth/RegisterForm.tsx`
- [X] T020 Integrate `useRegisterMutation` from RTK Query in `phase2/frontend/src/components/auth/RegisterForm.tsx`
- [X] T021 Add Sonner toast notifications (success: "Account created!", error: API message) in `phase2/frontend/src/components/auth/RegisterForm.tsx`
- [X] T022 Use LoadingButton with isLoading state in `phase2/frontend/src/components/auth/RegisterForm.tsx`
- [X] T023 Add inline validation error display (including password mismatch) in `phase2/frontend/src/components/auth/RegisterForm.tsx`

**Deliverables**:
- RegisterForm component with password confirmation
- Cross-field validation (password match)
- RTK Query integration
- Toast notifications

---

## Phase 6: Return URL Hook

**Purpose**: Create hook for extracting and using return URL after login

**Checkpoint**: useReturnUrl extracts `?returnTo=` from URL, provides default

- [X] T024 Create `phase2/frontend/src/hooks/useReturnUrl.ts` hook
- [X] T025 Extract `returnTo` from URL search params in `phase2/frontend/src/hooks/useReturnUrl.ts`
- [X] T026 Validate returnTo is internal path (starts with /) in `phase2/frontend/src/hooks/useReturnUrl.ts`
- [X] T027 Provide default redirect path (/tasks) if no returnTo in `phase2/frontend/src/hooks/useReturnUrl.ts`

**Deliverables**:
- `useReturnUrl` hook
- URL validation (prevent open redirect)
- Default fallback path

---

## Phase 7: Updated Auth Pages

**Purpose**: Update LoginPage and RegisterPage to use new form components

**Checkpoint**: `/login` and `/register` render styled forms with full functionality

- [X] T028 Update `phase2/frontend/src/pages/LoginPage.tsx` to use LoginForm component
- [X] T029 Integrate useReturnUrl hook in `phase2/frontend/src/pages/LoginPage.tsx`
- [X] T030 Handle successful login redirect using returnTo in `phase2/frontend/src/pages/LoginPage.tsx`
- [X] T031 [P] Update `phase2/frontend/src/pages/RegisterPage.tsx` to use RegisterForm component
- [X] T032 [P] Handle successful registration redirect to /tasks in `phase2/frontend/src/pages/RegisterPage.tsx`

**Deliverables**:
- LoginPage with LoginForm and return URL support
- RegisterPage with RegisterForm

---

## Phase 8: Protected Route Enhancement

**Purpose**: Update ProtectedRoute to pass returnTo in redirect

**Checkpoint**: Unauthenticated visit to `/tasks` → `/login?returnTo=/tasks` → login → back to `/tasks`

- [X] T033 Update `phase2/frontend/src/routes/ProtectedRoute.tsx` to capture current path
- [X] T034 Add returnTo search param to login redirect in `phase2/frontend/src/routes/ProtectedRoute.tsx`
- [X] T035 Verify full redirect flow works end-to-end

**Deliverables**:
- ProtectedRoute with returnTo support
- Complete redirect flow working

---

## Phase 9: Final Polish + Build Verification

**Purpose**: TypeScript verification and production build check

**Checkpoint**: `npm run build` → 0 errors, all forms functional

- [X] T036 Review and fix any TypeScript errors: `npx tsc --noEmit`
- [X] T037 Run production build: `npm run build` → verify dist/ created
- [X] T038 Manual test: Login flow with valid/invalid credentials
- [X] T039 Manual test: Register flow with validation errors
- [X] T040 Manual test: Return URL redirect flow

**Deliverables**:
- Zero TypeScript errors
- Production build successful
- All acceptance criteria verified

---

## Dependencies & Execution Order

### Phase Dependencies

```text
Phase 1: Dependencies + Setup ─────────────────────┐
                                                   │
                                                   ▼
Phase 2: Zod Validation Schemas ───────────────────┐
                                                   │
                                                   ▼
Phase 3: Loading Button Component ─────────────────┐
                                                   │
                                                   ▼
Phase 4: LoginForm Component ──────────────────────┐
         │                                         │
         ▼                                         │
Phase 5: RegisterForm Component ◄──────────────────┘
         │
         ▼
Phase 6: Return URL Hook ──────────────────────────┐
                                                   │
                                                   ▼
Phase 7: Updated Auth Pages ───────────────────────┐
                                                   │
                                                   ▼
Phase 8: Protected Route Enhancement ──────────────┐
                                                   │
                                                   ▼
Phase 9: Final Polish + Build
```

### Within Each Phase

- T001-T003: Sequential (setup must complete first)
- T004-T006: Sequential (schema depends on previous)
- T007-T009: Sequential (same file)
- T010-T016: Sequential (same component)
- T017-T023: Sequential (same component)
- T024-T027: Sequential (same hook)
- T028-T032: T028-T030 sequential, T031-T032 parallel (different pages)
- T033-T035: Sequential (same component)
- T036-T040: Sequential (verification)

### Parallel Opportunities

```bash
# Phase 7 (after T030):
T031: Update RegisterPage          [P]
T032: Handle register redirect     [P]
```

---

## Implementation Strategy

### Sequential Execution (Recommended)

1. Complete Phase 1 → Checkpoint: dependencies installed
2. Complete Phase 2 → Checkpoint: schemas export correctly
3. Complete Phase 3 → Checkpoint: LoadingButton renders spinner
4. Complete Phase 4 → Checkpoint: LoginForm works
5. Complete Phase 5 → Checkpoint: RegisterForm works
6. Complete Phase 6 → Checkpoint: useReturnUrl hook works
7. Complete Phase 7 → Checkpoint: Pages use new forms
8. Complete Phase 8 → Checkpoint: Return URL flow works
9. Complete Phase 9 → **DONE**

### MVP Validation Points

- **After Phase 5**: Core forms complete, can authenticate
- **After Phase 8**: Full redirect flow working
- **After Phase 9**: Production-ready authentication UI

---

## Files Created/Modified Summary

| File | Phase | Tasks | Status |
|------|-------|-------|--------|
| `package.json` | 1 | T001 | MODIFIED |
| `src/App.tsx` | 1 | T002 | MODIFIED |
| `src/lib/validations/auth.ts` | 2 | T004-T006 | NEW |
| `src/components/ui/loading-button.tsx` | 3 | T007-T009 | NEW |
| `src/components/auth/LoginForm.tsx` | 4 | T010-T016 | NEW |
| `src/components/auth/RegisterForm.tsx` | 5 | T017-T023 | NEW |
| `src/hooks/useReturnUrl.ts` | 6 | T024-T027 | NEW |
| `src/pages/LoginPage.tsx` | 7 | T028-T030 | MODIFIED |
| `src/pages/RegisterPage.tsx` | 7 | T031-T032 | MODIFIED |
| `src/routes/ProtectedRoute.tsx` | 8 | T033-T034 | MODIFIED |

**Total**: 40 tasks, 10 files (6 new, 4 modified)

---

## Notes

- [P] tasks = different files, no dependencies
- Phases must complete in order (each depends on previous)
- Checkpoints verify each phase before proceeding
- No tests in this phase (manual verification only)
- Backend must be running for API integration testing
