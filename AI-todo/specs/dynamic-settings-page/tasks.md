# Tasks — Dynamic Settings Page

**Date**: 2025-12-15
**Feature**: dynamic-settings-page
**Plan**: [plan.md](./plan.md)
**Spec**: [spec.md](./spec.md)

## Task Summary

| ID | Task | Est. | Status |
|----|------|------|--------|
| T001 | Backend: Model Extension + Migration | 2min | [X] |
| T002 | Backend: Schemas + Service Functions | 2min | [X] |
| T003 | Backend: Settings Endpoints | 2min | [X] |
| T004 | Frontend: Types + API + Validation | 2min | [X] |
| T005 | Frontend: SettingsPage + Routing | 2min | [X] |
| T006 | Frontend: Components + Theme Sync | 2min | [X] |

**Total**: 6 tasks (~12min)

---

## T001: Backend Model Extension + Migration

**Goal**: Add `theme` and `email_notifications` fields to User model

**Files**:
| File | Action |
|------|--------|
| `phase2/backend/app/models/user.py` | MODIFY |
| `phase2/backend/alembic/versions/20251215_000002_add_user_settings.py` | CREATE |

**Changes**:

```python
# app/models/user.py - ADD after avatar_url
theme: str = Field(
    default="system",
    max_length=20,
    nullable=False,
    description="Theme preference: light, dark, or system",
)
email_notifications: bool = Field(
    default=True,
    nullable=False,
    description="Email notification preference",
)
```

**Migration**:
```python
def upgrade() -> None:
    op.add_column('users', sa.Column('theme', sa.String(20), nullable=False, server_default='system'))
    op.add_column('users', sa.Column('email_notifications', sa.Boolean(), nullable=False, server_default='true'))

def downgrade() -> None:
    op.drop_column('users', 'email_notifications')
    op.drop_column('users', 'theme')
```

**Test Cases**:
- [ ] User model has `theme` field with default "system"
- [ ] User model has `email_notifications` field with default true
- [ ] Migration runs: `uv run alembic upgrade head`
- [ ] Rollback works: `uv run alembic downgrade -1`

**Acceptance**: Fields exist on User model, migration successful

---

## T002: Backend Schemas + Service Functions

**Goal**: Add Pydantic schemas and auth service functions for settings operations

**Files**:
| File | Action |
|------|--------|
| `phase2/backend/app/schemas/auth.py` | MODIFY |
| `phase2/backend/app/services/auth_service.py` | MODIFY |

**Schemas to Add**:

```python
# app/schemas/auth.py

class UserSettingsUpdate(BaseModel):
    """Schema for settings update request (partial update)."""
    theme: str | None = Field(default=None, pattern="^(light|dark|system)$")
    email_notifications: bool | None = None

class UserSettingsResponse(BaseModel):
    """Schema for settings response."""
    theme: str
    email_notifications: bool
    model_config = {"from_attributes": True}

class PasswordChangeRequest(BaseModel):
    """Schema for password change request."""
    current_password: str = Field(..., min_length=1)
    new_password: str = Field(..., min_length=8)

class MessageResponse(BaseModel):
    """Generic message response."""
    message: str
```

**Also**: Update `UserResponse` to include `theme` and `email_notifications`

**Service Functions**:

```python
# app/services/auth_service.py

async def update_user_settings(db: AsyncSession, user: User, data: UserSettingsUpdate) -> User:
    update_data = data.model_dump(exclude_unset=True)
    for field, value in update_data.items():
        setattr(user, field, value)
    db.add(user)
    await db.commit()
    await db.refresh(user)
    return user

async def change_password(db: AsyncSession, user: User, current: str, new: str) -> bool:
    if not verify_password(current, user.hashed_password):
        return False
    user.hashed_password = hash_password(new)
    db.add(user)
    await db.commit()
    return True

async def delete_user(db: AsyncSession, user: User) -> None:
    await db.delete(user)
    await db.commit()
```

**Test Cases**:
- [ ] UserSettingsUpdate validates theme enum (light/dark/system)
- [ ] UserResponse includes theme and email_notifications
- [ ] update_user_settings applies partial updates
- [ ] change_password returns False for wrong password
- [ ] delete_user removes user from database

**Acceptance**: All schemas validate correctly, services handle business logic

---

## T003: Backend Settings Endpoints

**Goal**: Add REST endpoints for settings, password change, and account deletion

**Files**:
| File | Action |
|------|--------|
| `phase2/backend/app/api/routes/auth.py` | MODIFY |

**Endpoints**:

```python
# app/api/routes/auth.py

@router.patch("/me/settings", response_model=UserSettingsResponse)
async def update_settings(
    data: UserSettingsUpdate,
    current_user: CurrentUser,
    db: DbSession,
) -> UserSettingsResponse:
    """Update current user's settings."""
    user = await auth_service.update_user_settings(db, current_user, data)
    return UserSettingsResponse.model_validate(user)


@router.post("/change-password", response_model=MessageResponse)
async def change_password(
    data: PasswordChangeRequest,
    current_user: CurrentUser,
    db: DbSession,
) -> MessageResponse:
    """Change current user's password."""
    success = await auth_service.change_password(
        db, current_user, data.current_password, data.new_password
    )
    if not success:
        raise HTTPException(status_code=401, detail="Current password is incorrect")
    return MessageResponse(message="Password changed successfully")


@router.delete("/me", status_code=status.HTTP_204_NO_CONTENT)
async def delete_account(
    current_user: CurrentUser,
    db: DbSession,
) -> None:
    """Delete current user's account and all data."""
    await auth_service.delete_user(db, current_user)
```

**Test Cases**:
- [ ] `PATCH /auth/me/settings` returns 200 with updated settings
- [ ] `POST /auth/change-password` returns 200 for correct password
- [ ] `POST /auth/change-password` returns 401 for wrong password
- [ ] `DELETE /auth/me` returns 204 and removes user
- [ ] All endpoints require authentication (401 without token)

**Acceptance**: All endpoints work per OpenAPI spec in contracts/settings-api.yaml

---

## T004: Frontend Types + API + Validation

**Goal**: Add TypeScript types, RTK Query endpoints, and Zod validation

**Files**:
| File | Action |
|------|--------|
| `phase2/frontend/src/types/settings.ts` | CREATE |
| `phase2/frontend/src/services/settingsApi.ts` | CREATE |
| `phase2/frontend/src/lib/validations/settings.ts` | CREATE |

**Types**:

```typescript
// src/types/settings.ts
export type Theme = 'light' | 'dark' | 'system';

export interface UserSettings {
  theme: Theme;
  email_notifications: boolean;
}

export interface UserSettingsUpdate {
  theme?: Theme;
  email_notifications?: boolean;
}

export interface PasswordChangeRequest {
  current_password: string;
  new_password: string;
}

export interface MessageResponse {
  message: string;
}
```

**RTK Query Endpoints**:

```typescript
// src/services/settingsApi.ts
export const settingsApi = api.injectEndpoints({
  endpoints: (builder) => ({
    updateSettings: builder.mutation<UserSettings, UserSettingsUpdate>({
      query: (data) => ({ url: '/auth/me/settings', method: 'PATCH', body: data }),
      invalidatesTags: ['User'],
    }),
    changePassword: builder.mutation<MessageResponse, PasswordChangeRequest>({
      query: (data) => ({ url: '/auth/change-password', method: 'POST', body: data }),
    }),
    deleteAccount: builder.mutation<void, void>({
      query: () => ({ url: '/auth/me', method: 'DELETE' }),
    }),
  }),
});

export const { useUpdateSettingsMutation, useChangePasswordMutation, useDeleteAccountMutation } = settingsApi;
```

**Zod Validation**:

```typescript
// src/lib/validations/settings.ts
export const passwordChangeSchema = z.object({
  currentPassword: z.string().min(1, 'Current password is required'),
  newPassword: z.string().min(8, 'Password must be at least 8 characters'),
  confirmPassword: z.string().min(1, 'Please confirm your password'),
}).refine(data => data.newPassword === data.confirmPassword, {
  message: 'Passwords do not match',
  path: ['confirmPassword'],
});

export type PasswordChangeFormData = z.infer<typeof passwordChangeSchema>;
```

**Test Cases**:
- [ ] Types match backend schemas
- [ ] RTK hooks export correctly
- [ ] Zod schema validates password matching
- [ ] Zod schema rejects short passwords

**Acceptance**: Types compile, hooks available, validation works

---

## T005: Frontend SettingsPage + Routing

**Goal**: Create settings page shell and wire up routing

**Files**:
| File | Action |
|------|--------|
| `phase2/frontend/src/pages/SettingsPage.tsx` | CREATE |
| `phase2/frontend/src/routes/index.tsx` | MODIFY |
| `phase2/frontend/src/App.tsx` | MODIFY |
| `phase2/frontend/src/components/layout/Header.tsx` | MODIFY |

**Route**:
```typescript
// src/routes/index.tsx
SETTINGS: '/settings',
```

**App.tsx**:
```tsx
import { SettingsPage } from '@/pages/SettingsPage';

<Route
  path={ROUTES.SETTINGS}
  element={
    <ProtectedRoute>
      <SettingsPage />
    </ProtectedRoute>
  }
/>
```

**SettingsPage Shell**:
```tsx
// src/pages/SettingsPage.tsx
export function SettingsPage() {
  return (
    <div className="container mx-auto py-8 space-y-8">
      <h1 className="text-2xl font-bold">Settings</h1>
      {/* ThemeSettings */}
      {/* NotificationSettings */}
      {/* PasswordChangeForm */}
      {/* DeleteAccountSection */}
    </div>
  );
}
```

**Header Update**: Add Settings link in dropdown menu

**Test Cases**:
- [ ] `/settings` route renders SettingsPage
- [ ] Route is protected (redirects if not authenticated)
- [ ] Header has Settings link
- [ ] Settings link navigates correctly

**Acceptance**: Settings page accessible at /settings with auth protection

---

## T006: Frontend Components + Theme Sync

**Goal**: Build settings components and wire theme sync

**Files**:
| File | Action |
|------|--------|
| `phase2/frontend/src/components/settings/ThemeSettings.tsx` | CREATE |
| `phase2/frontend/src/components/settings/NotificationSettings.tsx` | CREATE |
| `phase2/frontend/src/components/settings/PasswordChangeForm.tsx` | CREATE |
| `phase2/frontend/src/components/settings/DeleteAccountDialog.tsx` | CREATE |
| `phase2/frontend/src/pages/SettingsPage.tsx` | MODIFY |

**Install shadcn Switch**:
```bash
cd phase2/frontend && npx shadcn@latest add switch alert-dialog
```

**Components**:

1. **ThemeSettings**: Radio group for light/dark/system
   - Syncs with Zustand theme store
   - Calls `updateSettings({ theme })` on change

2. **NotificationSettings**: Switch toggle
   - Calls `updateSettings({ email_notifications })` on toggle

3. **PasswordChangeForm**: React Hook Form + Zod
   - Current password, new password, confirm password fields
   - Password visibility toggles
   - Calls `changePassword()` mutation

4. **DeleteAccountDialog**: AlertDialog
   - Requires typing "DELETE" to enable button
   - Calls `deleteAccount()` mutation
   - Redirects to home on success

**Theme Sync Logic**:
```typescript
// In App.tsx or ThemeSettings
const { data: user } = useGetCurrentUserQuery();

useEffect(() => {
  if (user?.theme) {
    setTheme(user.theme as Theme);
  }
}, [user?.theme, setTheme]);
```

**Test Cases**:
- [ ] Theme changes apply immediately (Zustand)
- [ ] Theme syncs to backend on change
- [ ] Notification toggle updates backend
- [ ] Password form validates and shows errors
- [ ] Delete dialog requires "DELETE" confirmation
- [ ] All actions show toast notifications

**Acceptance**: All settings functional, theme persists across sessions

---

## Execution Order

```
T001 → T002 → T003 (Backend)
         ↓
T004 → T005 → T006 (Frontend)
```

**Dependencies**:
- T002 depends on T001 (model fields must exist)
- T003 depends on T002 (schemas must exist)
- T004-T006 depend on T003 (backend must be ready)

---

## Quick Commands

```bash
# Backend - run migration
cd phase2/backend && uv run alembic upgrade head

# Backend - run tests
cd phase2/backend && uv run pytest tests/unit/test_settings.py -v

# Frontend - install components
cd phase2/frontend && npx shadcn@latest add switch alert-dialog

# Frontend - type check
cd phase2/frontend && npm run type-check
```
