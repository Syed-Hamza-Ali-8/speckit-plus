# Implementation Plan: Dynamic Profile Page

**Branch**: `main` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/dynamic-profile-page/spec.md`

## Summary

Implement a user profile page displaying real user data from `/auth/me`, with an edit profile form using `PATCH /auth/me`, task statistics cards, and avatar support. Uses existing RTK Query patterns with shadcn Form components.

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript 5.x (frontend)
**Primary Dependencies**: FastAPI, SQLModel, React 18, RTK Query, shadcn/ui, Zod
**Storage**: Neon DB (PostgreSQL via SQLModel)
**Testing**: pytest (backend), Vitest (frontend)
**Target Platform**: Web (responsive)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: < 200ms profile load, optimistic UI updates
**Constraints**: Avatar as base64 (MVP), derive task stats from existing endpoint
**Scale/Scope**: Single user profile view, 2 editable fields

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Rule | Status | Notes |
|------|--------|-------|
| Spec exists in `/specs/` | ✅ PASS | `specs/dynamic-profile-page/spec.md` |
| Phase II technologies only | ✅ PASS | FastAPI, Next.js/React, SQLModel, JWT |
| Clean Architecture | ✅ PASS | Service layer, schema separation |
| TypeScript strict mode | ✅ PASS | Existing frontend config |
| Type hints (Python) | ✅ PASS | All schemas typed |
| JWT authentication | ✅ PASS | Uses existing CurrentUser dependency |
| Minimum 80% coverage | ⚠️ CHECK | Tests defined in tasks |

## Project Structure

### Documentation (this feature)

```text
specs/dynamic-profile-page/
├── spec.md              # Feature requirements
├── plan.md              # This file
├── research.md          # Phase 0 research decisions
├── data-model.md        # Entity definitions
├── quickstart.md        # Implementation guide
├── contracts/           # API contracts
│   └── profile-api.yaml # OpenAPI spec for PATCH /auth/me
└── tasks.md             # Implementation tasks (via /sp.tasks)
```

### Source Code (repository root)

```text
phase2/
├── backend/
│   ├── app/
│   │   ├── api/routes/auth.py      # MODIFY: Add PATCH /me endpoint
│   │   ├── schemas/auth.py         # MODIFY: Add UserUpdate schema
│   │   ├── services/auth_service.py # MODIFY: Add update_user_profile
│   │   └── models/user.py          # MODIFY: Add avatar_url field
│   ├── alembic/versions/           # ADD: Migration for avatar_url
│   └── tests/
│       └── unit/test_profile.py    # ADD: Profile update tests
│
└── frontend/
    ├── src/
    │   ├── pages/ProfilePage.tsx       # ADD: Main profile page
    │   ├── components/profile/         # ADD: Profile components
    │   │   ├── ProfileHeader.tsx
    │   │   ├── ProfileForm.tsx
    │   │   ├── AvatarUpload.tsx
    │   │   └── TaskStatsCards.tsx
    │   ├── services/userApi.ts         # ADD: User RTK Query endpoints
    │   ├── types/user.ts               # ADD: User type definitions
    │   ├── lib/validations/profile.ts  # ADD: Zod schema
    │   └── routes/index.tsx            # MODIFY: Add PROFILE route
    └── tests/
        └── ProfilePage.test.tsx        # ADD: Component tests
```

**Structure Decision**: Web application structure with clear backend/frontend separation. Follows existing patterns from auth and tasks modules.

---

## Implementation Steps (6 Atomic Tasks)

### Step 1: Backend Schema & Service

**Goal**: Add UserUpdate schema and service function for profile updates

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/backend/app/schemas/auth.py` | ADD `UserUpdate` schema |
| `phase2/backend/app/services/auth_service.py` | ADD `update_user_profile()` |

**Schema Definition**:
```python
class UserUpdate(BaseModel):
    first_name: str | None = Field(default=None, max_length=100)
    last_name: str | None = Field(default=None, max_length=100)
    avatar_url: str | None = Field(default=None)
```

**Tests**:
- Unit test for `update_user_profile` service function
- Validates partial updates (only provided fields change)

**Acceptance Criteria**:
- [ ] `UserUpdate` schema accepts optional first_name, last_name, avatar_url
- [ ] `update_user_profile` updates only provided fields
- [ ] Service returns updated User object

---

### Step 2: Backend PATCH Endpoint

**Goal**: Implement `PATCH /auth/me` endpoint

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/backend/app/api/routes/auth.py` | ADD `PATCH /me` route |

**Endpoint Definition**:
```python
@router.patch("/me", response_model=UserResponse)
async def update_current_user_profile(
    data: UserUpdate,
    current_user: CurrentUser,
    db: DbSession,
) -> UserResponse:
    user = await auth_service.update_user_profile(db, current_user, data)
    return UserResponse.model_validate(user)
```

**Tests**:
- Integration test: PATCH with valid data returns 200
- Integration test: PATCH without auth returns 401
- Integration test: PATCH with invalid data returns 422

**Acceptance Criteria**:
- [ ] `PATCH /auth/me` requires authentication (401 without token)
- [ ] Updates first_name, last_name, avatar_url fields
- [ ] Returns updated `UserResponse`
- [ ] Rate limited (optional)

---

### Step 3: Frontend Types & API Layer

**Goal**: Add TypeScript types and RTK Query endpoints for user profile

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/frontend/src/types/user.ts` | CREATE User types |
| `phase2/frontend/src/services/userApi.ts` | CREATE RTK Query endpoints |
| `phase2/frontend/src/lib/validations/profile.ts` | CREATE Zod schema |

**API Endpoints**:
```typescript
useGetCurrentUserQuery()   // GET /auth/me
useUpdateProfileMutation() // PATCH /auth/me
```

**Tests**:
- Type compilation (no TypeScript errors)
- RTK Query hooks export correctly

**Acceptance Criteria**:
- [ ] `User` interface matches backend `UserResponse`
- [ ] `useGetCurrentUserQuery` provides User tag for cache
- [ ] `useUpdateProfileMutation` invalidates User tag
- [ ] Zod `profileSchema` validates max 100 chars

---

### Step 4: Profile Page & Routing

**Goal**: Create ProfilePage component and add `/profile` route

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/frontend/src/pages/ProfilePage.tsx` | CREATE main page |
| `phase2/frontend/src/routes/index.tsx` | ADD PROFILE route |
| `phase2/frontend/src/App.tsx` | ADD ProfilePage route |
| `phase2/frontend/src/components/layout/Header.tsx` | ADD Profile link |

**Component Structure**:
```tsx
<ProfilePage>
  <ProfileHeader user={user} />
  <TaskStatsCards />
  <ProfileForm user={user} />
</ProfilePage>
```

**Tests**:
- Route renders ProfilePage at `/profile`
- Redirects to login when unauthenticated

**Acceptance Criteria**:
- [ ] `/profile` route exists and is protected
- [ ] ProfilePage fetches user data on mount
- [ ] Header navigation includes Profile link
- [ ] Loading/error states handled

---

### Step 5: Profile Components (Header + Stats)

**Goal**: Build ProfileHeader and TaskStatsCards components

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/frontend/src/components/profile/ProfileHeader.tsx` | CREATE |
| `phase2/frontend/src/components/profile/TaskStatsCards.tsx` | CREATE |
| `phase2/frontend/src/components/profile/AvatarDisplay.tsx` | CREATE |

**shadcn Dependencies**:
```bash
npx shadcn@latest add avatar card
```

**Component Features**:
- ProfileHeader: Avatar, display name, email, member since
- TaskStatsCards: Total/Completed/Pending with icons
- AvatarDisplay: Fallback to initials when no avatar

**Tests**:
- Components render with mock user data
- Stats calculated correctly from tasks

**Acceptance Criteria**:
- [ ] ProfileHeader displays all user fields
- [ ] AvatarDisplay shows initials fallback
- [ ] TaskStatsCards computes stats from tasks
- [ ] Cards use shadcn Card component
- [ ] Mobile responsive grid

---

### Step 6: Profile Edit Form

**Goal**: Build ProfileForm with edit dialog and form submission

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/frontend/src/components/profile/ProfileForm.tsx` | CREATE |
| `phase2/frontend/src/components/profile/AvatarUpload.tsx` | CREATE (optional) |

**shadcn Dependencies**:
```bash
npx shadcn@latest add dialog form
```

**Component Features**:
- Dialog modal for editing
- React Hook Form + Zod validation
- Loading state during submission
- Toast notifications

**Tests**:
- Dialog opens/closes correctly
- Form validates field lengths
- Submission calls mutation
- Success/error toasts display

**Acceptance Criteria**:
- [ ] Edit Profile button opens dialog
- [ ] Form pre-fills current values
- [ ] Zod validation enforces max 100 chars
- [ ] Submit calls `useUpdateProfileMutation`
- [ ] Toast on success: "Profile updated!"
- [ ] Toast on error: "Failed to update profile"
- [ ] Dialog closes on success
- [ ] Cache refreshes automatically

---

## Dependency Graph

```
Step 1 ──► Step 2 ──► Step 3 ──► Step 4 ──► Step 5 ──► Step 6
(Schema)   (Route)    (Types)    (Page)    (Header)   (Form)
                         │
                         └──────────────────────────────┘
                              (Can parallelize 5 & 6)
```

## Risk Analysis

| Risk | Mitigation |
|------|------------|
| Avatar base64 too large for DB | Limit to 2MB, compress on frontend |
| Task stats slow with many tasks | Pagination limit 1000, or add stats endpoint later |
| Cache invalidation issues | RTK Query tags handle automatically |

## Complexity Tracking

No constitution violations requiring justification.

---

## Next Steps

1. Run `/sp.tasks` to generate detailed task file with test cases
2. Implement Step 1-2 (backend) first
3. Implement Step 3-6 (frontend) after backend ready
4. Run `/sp.analyze` for cross-artifact consistency check
