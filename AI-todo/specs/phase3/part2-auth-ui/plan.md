# Implementation Plan: Phase 3 Part 2 - Authentication UI

**Feature**: Production-ready authentication forms with validation, toast notifications, and smart redirects
**Prerequisites**: Phase 3 Part 1 (Frontend Setup) complete
**Output**: Fully functional login/register forms integrated with backend API

---

## 1. Implementation Phases

### Phase 1: Dependencies + Zod Schemas

**Purpose**: Install required packages and set up validation schemas

**Tasks**:
1. Install dependencies: `npm install react-hook-form @hookform/resolvers zod sonner`
2. Create `src/lib/validations/auth.ts` with Zod schemas (loginSchema, registerSchema)
3. Verify TypeScript compilation passes

**Deliverables**:
- New dependencies in package.json
- `src/lib/validations/auth.ts` with typed schemas

**Checkpoint**: `npx tsc --noEmit` passes

---

### Phase 2: Sonner Toast Integration

**Purpose**: Set up toast notifications system

**Tasks**:
1. Add `<Toaster />` component from Sonner to App.tsx
2. Create toast utility functions in `src/lib/toast.ts`
3. Test toast notifications work (success, error types)

**Deliverables**:
- Sonner Toaster integrated in App.tsx
- Toast utility functions for auth events

**Checkpoint**: Toast notification renders when triggered

---

### Phase 3: Loading Button Component

**Purpose**: Create reusable loading button component

**Tasks**:
1. Create `src/components/ui/loading-button.tsx`
2. Integrate spinner icon from Lucide React
3. Support loading state, disabled state, and custom loading text

**Deliverables**:
- `LoadingButton` component with spinner

**Checkpoint**: Button shows spinner when loading prop is true

---

### Phase 4: Login/Register Forms

**Purpose**: Build form components with React Hook Form + Zod

**Tasks**:
1. Create `src/components/auth/LoginForm.tsx`:
   - React Hook Form with Zod resolver
   - Email and password fields with inline validation
   - Integration with `useLoginMutation`
   - Toast on success/error
   - Loading state during submission

2. Create `src/components/auth/RegisterForm.tsx`:
   - React Hook Form with Zod resolver
   - Email, password, confirm password fields
   - Cross-field validation (password match)
   - Integration with `useRegisterMutation`
   - Toast on success/error
   - Loading state during submission

3. Update `src/pages/LoginPage.tsx`:
   - Use LoginForm component
   - Extract returnTo from URL query params
   - Handle redirect after success

4. Update `src/pages/RegisterPage.tsx`:
   - Use RegisterForm component
   - Handle redirect after success

**Deliverables**:
- LoginForm component
- RegisterForm component
- Updated LoginPage and RegisterPage

**Checkpoint**: Forms render, validate, and submit to API

---

### Phase 5: Return URL + Protected Route

**Purpose**: Implement smart redirect after login

**Tasks**:
1. Create `src/hooks/useReturnUrl.ts`:
   - Extract returnTo from URL search params
   - Provide default redirect path (/tasks)

2. Update `src/routes/ProtectedRoute.tsx`:
   - Pass current path as returnTo to login redirect
   - Use Navigate with search params

3. Update LoginForm/RegisterForm:
   - Use useReturnUrl hook
   - Navigate to returnTo after successful auth

4. Final verification and TypeScript check

**Deliverables**:
- useReturnUrl hook
- Updated ProtectedRoute with returnTo support
- Complete redirect flow

**Checkpoint**: Visit `/tasks` → redirect to `/login?returnTo=/tasks` → login → back to `/tasks`

---

## 2. Dependency Graph

```
Phase 1: Dependencies + Zod Schemas
         │
         ▼
Phase 2: Sonner Toast Integration
         │
         ▼
Phase 3: Loading Button Component
         │
         ▼
Phase 4: Login/Register Forms ──────────────────────┐
         │                                           │
         ▼                                           │
Phase 5: Return URL + Protected Route ◄─────────────┘
```

**Critical Path**: Phases must execute sequentially (each depends on previous)

---

## 3. Complete File Structure

```
phase2/frontend/
├── src/
│   ├── components/
│   │   ├── auth/
│   │   │   ├── LoginForm.tsx        # NEW - RHF + Zod + RTK login
│   │   │   └── RegisterForm.tsx     # NEW - RHF + Zod + RTK register
│   │   ├── layout/
│   │   │   ├── Layout.tsx           # EXISTING
│   │   │   ├── Header.tsx           # EXISTING
│   │   │   └── ThemeToggle.tsx      # EXISTING
│   │   └── ui/
│   │       ├── button.tsx           # EXISTING
│   │       ├── card.tsx             # EXISTING
│   │       ├── input.tsx            # EXISTING
│   │       ├── label.tsx            # EXISTING
│   │       └── loading-button.tsx   # NEW - Button with spinner
│   ├── hooks/
│   │   ├── useAuth.ts               # EXISTING
│   │   ├── useTheme.ts              # EXISTING
│   │   └── useReturnUrl.ts          # NEW - Return URL extraction
│   ├── lib/
│   │   ├── store.ts                 # EXISTING
│   │   ├── utils.ts                 # EXISTING
│   │   ├── toast.ts                 # NEW - Toast utilities
│   │   └── validations/
│   │       └── auth.ts              # NEW - Zod schemas
│   ├── pages/
│   │   ├── LoginPage.tsx            # UPDATED - Use LoginForm
│   │   ├── RegisterPage.tsx         # UPDATED - Use RegisterForm
│   │   └── TasksPage.tsx            # EXISTING
│   ├── routes/
│   │   ├── index.tsx                # EXISTING
│   │   └── ProtectedRoute.tsx       # UPDATED - Add returnTo
│   ├── services/
│   │   ├── api.ts                   # EXISTING
│   │   ├── authApi.ts               # EXISTING
│   │   └── taskApi.ts               # EXISTING
│   ├── stores/
│   │   └── uiStore.ts               # EXISTING
│   ├── types/
│   │   ├── auth.ts                  # EXISTING
│   │   └── task.ts                  # EXISTING
│   ├── App.tsx                      # UPDATED - Add Sonner Toaster
│   ├── main.tsx                     # EXISTING
│   └── index.css                    # EXISTING
└── package.json                     # UPDATED - New dependencies
```

---

## 4. Success Criteria

| # | Criterion | Verification |
|---|-----------|--------------|
| 1 | Dependencies installed | `npm ls react-hook-form @hookform/resolvers zod sonner` |
| 2 | Zod schemas compile | `npx tsc --noEmit` passes |
| 3 | Sonner Toaster renders | Toast appears on trigger |
| 4 | LoadingButton shows spinner | Button has spinner when loading=true |
| 5 | LoginForm validates email | Invalid email shows error |
| 6 | LoginForm validates password | <8 chars shows error |
| 7 | RegisterForm validates password match | Mismatch shows error |
| 8 | LoginForm calls RTK mutation | Network tab shows API call |
| 9 | Success toast shows | Login → "Welcome back!" toast |
| 10 | Error toast shows | Invalid creds → error toast |
| 11 | JWT stored on login | localStorage has token |
| 12 | Redirect to /tasks | Login success → navigates to /tasks |
| 13 | Return URL works | /login?returnTo=/tasks/123 → login → /tasks/123 |
| 14 | ProtectedRoute adds returnTo | /tasks (unauth) → /login?returnTo=/tasks |
| 15 | Production build passes | `npm run build` succeeds |

---

## 5. Dependencies

### New Packages

```json
{
  "dependencies": {
    "react-hook-form": "^7.54.0",
    "@hookform/resolvers": "^3.9.0",
    "zod": "^3.24.0",
    "sonner": "^1.7.0"
  }
}
```

### Existing Dependencies (from Part 1)

- `@reduxjs/toolkit` - RTK Query mutations
- `react-redux` - Redux provider
- `react-router-dom` - Routing and navigation
- `lucide-react` - Icons (Loader2 for spinner)
- shadcn/ui components - Button, Input, Label, Card

---

## 6. Risk Mitigation

| Risk | Mitigation |
|------|------------|
| RHF + RTK Query conflict | Use RHF only for form state, RTK for API calls |
| Zod schema mismatch | Use same schemas client and server side |
| Toast z-index issues | Configure Sonner with proper z-index |
| Return URL XSS | Validate returnTo is internal path only |

---

## 7. Constitution Compliance

| Rule | Status | Notes |
|------|--------|-------|
| Spec-Driven Development | COMPLIANT | spec.md exists |
| Phase II Technology Stack | COMPLIANT | React/Vite allowed per constitution |
| TypeScript Strict Mode | COMPLIANT | Enabled in tsconfig.json |
| Clean Architecture | COMPLIANT | Clear separation: components, hooks, services |
| Testing Requirements | PENDING | Tests planned in Phase 5 |
| Interface Contracts | COMPLIANT | TypeScript interfaces defined |

---

## 8. Related Documents

- [spec.md](./spec.md) - Formal specification
- [Phase 3 Part 1 plan.md](../part1-frontend-setup/plan.md) - Frontend setup plan
- [data-model.md](../part1-frontend-setup/data-model.md) - TypeScript type definitions
