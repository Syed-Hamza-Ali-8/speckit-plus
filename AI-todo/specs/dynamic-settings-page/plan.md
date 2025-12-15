# Implementation Plan: Dynamic Settings Page

**Branch**: `main` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/dynamic-settings-page/spec.md`

## Summary

Implement a user settings page with theme preferences synced to backend, secure password change functionality, email notification toggle, and account deletion with confirmation dialog. Uses existing RTK Query patterns with shadcn Form components.

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript 5.x (frontend)
**Primary Dependencies**: FastAPI, SQLModel, React 18, RTK Query, shadcn/ui, Zod
**Storage**: Neon DB (PostgreSQL via SQLModel)
**Testing**: pytest (backend), Vitest (frontend)
**Target Platform**: Web (responsive)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: < 200ms settings load, immediate theme apply
**Constraints**: Rate limit password change (5/hour), hard delete for GDPR
**Scale/Scope**: Single user settings, 4 distinct features

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Rule | Status | Notes |
|------|--------|-------|
| Spec exists in `/specs/` | ✅ PASS | `specs/dynamic-settings-page/spec.md` |
| Phase II technologies only | ✅ PASS | FastAPI, React, SQLModel, JWT |
| Clean Architecture | ✅ PASS | Service layer, schema separation |
| TypeScript strict mode | ✅ PASS | Existing frontend config |
| Type hints (Python) | ✅ PASS | All schemas typed |
| JWT authentication | ✅ PASS | Uses existing CurrentUser dependency |
| Minimum 80% coverage | ⚠️ CHECK | Tests defined in tasks |

## Project Structure

### Documentation (this feature)

```text
specs/dynamic-settings-page/
├── spec.md              # Feature requirements
├── plan.md              # This file
├── research.md          # Phase 0 research decisions
├── data-model.md        # Entity definitions
├── quickstart.md        # Implementation guide
├── contracts/           # API contracts
│   └── settings-api.yaml
└── tasks.md             # Implementation tasks (via /sp.tasks)
```

### Source Code (repository root)

```text
phase2/
├── backend/
│   ├── app/
│   │   ├── api/routes/auth.py      # MODIFY: Add settings/password/delete endpoints
│   │   ├── schemas/auth.py         # MODIFY: Add settings schemas
│   │   ├── services/auth_service.py # MODIFY: Add settings/password/delete functions
│   │   └── models/user.py          # MODIFY: Add theme, email_notifications fields
│   ├── alembic/versions/           # ADD: Migration for new fields
│   └── tests/
│       └── unit/test_settings.py   # ADD: Settings tests
│
└── frontend/
    ├── src/
    │   ├── pages/SettingsPage.tsx       # ADD: Main settings page
    │   ├── components/settings/         # ADD: Settings components
    │   │   ├── ThemeSettings.tsx
    │   │   ├── NotificationSettings.tsx
    │   │   ├── PasswordChangeForm.tsx
    │   │   └── DeleteAccountDialog.tsx
    │   ├── services/settingsApi.ts      # ADD: RTK Query endpoints
    │   ├── types/settings.ts            # ADD: Settings type definitions
    │   ├── lib/validations/settings.ts  # ADD: Zod schemas
    │   └── routes/index.tsx             # MODIFY: Add SETTINGS route
    └── tests/
        └── SettingsPage.test.tsx        # ADD: Component tests
```

**Structure Decision**: Web application structure with clear backend/frontend separation. Follows existing patterns from profile page.

---

## Implementation Steps (8-Step Roadmap)

### Step 1: Backend Model & Migration

**Goal**: Extend User model with settings fields and create migration

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/backend/app/models/user.py` | ADD `theme`, `email_notifications` fields |
| `phase2/backend/alembic/versions/` | ADD migration script |

**Model Extension**:
```python
theme: str = Field(default="system", max_length=20, nullable=False)
email_notifications: bool = Field(default=True, nullable=False)
```

**Acceptance Criteria**:
- [ ] User model has `theme` field (default: "system")
- [ ] User model has `email_notifications` field (default: true)
- [ ] Migration runs successfully

---

### Step 2: Backend Schemas

**Goal**: Add Pydantic schemas for settings operations

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/backend/app/schemas/auth.py` | ADD `UserSettingsUpdate`, `UserSettingsResponse`, `PasswordChangeRequest`, `MessageResponse` |

**Schemas**:
- `UserSettingsUpdate`: partial update (theme?, email_notifications?)
- `UserSettingsResponse`: current settings
- `PasswordChangeRequest`: current_password, new_password
- `MessageResponse`: generic success message

**Also**: Update `UserResponse` to include theme and email_notifications

**Acceptance Criteria**:
- [ ] All schemas defined with proper validation
- [ ] UserResponse includes settings fields
- [ ] Theme field validates against enum (light/dark/system)

---

### Step 3: Backend Service Functions

**Goal**: Implement service layer functions for settings, password change, and account deletion

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/backend/app/services/auth_service.py` | ADD `update_user_settings()`, `change_password()`, `delete_user()` |

**Functions**:
```python
async def update_user_settings(db, user, data: UserSettingsUpdate) -> User
async def change_password(db, user, current: str, new: str) -> bool
async def delete_user(db, user) -> None
```

**Acceptance Criteria**:
- [ ] `update_user_settings` applies partial updates
- [ ] `change_password` verifies current password before changing
- [ ] `delete_user` removes user and cascades to tasks

---

### Step 4: Backend Endpoints

**Goal**: Implement REST endpoints for settings operations

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/backend/app/api/routes/auth.py` | ADD `PATCH /me/settings`, `POST /change-password`, `DELETE /me` |

**Endpoints**:
- `PATCH /auth/me/settings` - Update theme/notifications
- `POST /auth/change-password` - Change password (rate limited 5/hour)
- `DELETE /auth/me` - Delete account

**Acceptance Criteria**:
- [ ] Settings endpoint returns updated settings
- [ ] Password change returns 401 for wrong password
- [ ] Password change is rate limited
- [ ] Delete returns 204 and removes all user data

---

### Step 5: Frontend Types & API Layer

**Goal**: Add TypeScript types and RTK Query endpoints

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/frontend/src/types/settings.ts` | CREATE settings types |
| `phase2/frontend/src/services/settingsApi.ts` | CREATE RTK Query endpoints |
| `phase2/frontend/src/lib/validations/settings.ts` | CREATE Zod schemas |

**API Hooks**:
```typescript
useUpdateSettingsMutation()   // PATCH /auth/me/settings
useChangePasswordMutation()   // POST /auth/change-password
useDeleteAccountMutation()    // DELETE /auth/me
```

**Acceptance Criteria**:
- [ ] Types match backend schemas
- [ ] RTK hooks invalidate User cache on settings update
- [ ] Zod schema validates password change form

---

### Step 6: Settings Page & Routing

**Goal**: Create settings page and add route

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/frontend/src/pages/SettingsPage.tsx` | CREATE main settings page |
| `phase2/frontend/src/routes/index.tsx` | ADD SETTINGS route |
| `phase2/frontend/src/App.tsx` | ADD SettingsPage route |
| `phase2/frontend/src/components/layout/Header.tsx` | UPDATE Settings link |

**Page Structure**:
```tsx
<SettingsPage>
  <ThemeSettings />
  <NotificationSettings />
  <PasswordChangeForm />
  <DeleteAccountSection />
</SettingsPage>
```

**Acceptance Criteria**:
- [ ] Settings page renders at `/settings`
- [ ] Route is protected (requires auth)
- [ ] Header Settings button navigates to settings

---

### Step 7: Settings Components

**Goal**: Build individual settings sections

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/frontend/src/components/settings/ThemeSettings.tsx` | CREATE |
| `phase2/frontend/src/components/settings/NotificationSettings.tsx` | CREATE |
| `phase2/frontend/src/components/settings/PasswordChangeForm.tsx` | CREATE |
| `phase2/frontend/src/components/settings/DeleteAccountDialog.tsx` | CREATE |

**shadcn Dependencies**:
```bash
npx shadcn@latest add switch alert-dialog
```

**Components**:
- **ThemeSettings**: Radio group for light/dark/system
- **NotificationSettings**: Switch toggle
- **PasswordChangeForm**: React Hook Form + Zod + password visibility
- **DeleteAccountDialog**: AlertDialog with "DELETE" confirmation

**Acceptance Criteria**:
- [ ] Theme changes apply immediately
- [ ] Notification toggle syncs to backend
- [ ] Password form validates and shows errors
- [ ] Delete dialog requires typing "DELETE"
- [ ] All actions show toast notifications

---

### Step 8: Theme Sync & Integration

**Goal**: Sync theme from backend to Zustand store on login

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/frontend/src/hooks/useAuth.ts` or `App.tsx` | MODIFY to sync theme on user load |
| `phase2/frontend/src/components/settings/ThemeSettings.tsx` | MODIFY to sync to backend on change |

**Sync Flow**:
1. User logs in → fetch `/auth/me` → get `theme`
2. Apply theme to Zustand store → UI updates
3. User changes theme → update Zustand (immediate) → PATCH to backend
4. On logout → keep theme in localStorage as fallback

**Acceptance Criteria**:
- [ ] Theme from backend applied on login
- [ ] Theme persists across sessions
- [ ] Theme changes sync to backend
- [ ] Logout clears session but preserves theme preference

---

## Dependency Graph

```
Step 1 ──► Step 2 ──► Step 3 ──► Step 4 (Backend)
                                    ↓
                              ┌─────┴─────┐
                              ↓           ↓
                          Step 5       (Backend Ready)
                              ↓
                          Step 6
                              ↓
                          Step 7
                              ↓
                          Step 8
```

### Parallel Opportunities

```bash
# After Step 4 completes, frontend work can begin
# Step 5-8 are sequential (frontend)
# Within Step 7, components can be built in parallel
```

---

## Risk Analysis

| Risk | Mitigation |
|------|------------|
| Rate limit bypass | slowapi handles at middleware level |
| Theme flash on load | Apply theme from localStorage first, then sync |
| Cascade delete fails | FK constraints with ON DELETE CASCADE |
| Password brute force | Rate limit + Argon2id slow hashing |

## Complexity Tracking

No constitution violations requiring justification.

---

## Next Steps

1. Run `/sp.tasks` to generate detailed task file
2. Implement Step 1-4 (backend) first
3. Run migration
4. Implement Step 5-8 (frontend)
5. Run `/sp.analyze` for cross-artifact consistency check
