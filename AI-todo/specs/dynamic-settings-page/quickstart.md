# Quickstart — Dynamic Settings Page

**Date**: 2025-12-15
**Feature**: dynamic-settings-page

## Prerequisites

- Node.js 18+
- Python 3.11+
- Running backend (`uvicorn app.main:app`)
- Running frontend (`npm run dev`)
- Authenticated user (JWT token)

## Implementation Order

```
Step 1-2: Backend Model & Migration
         ↓
Step 3-4: Backend Endpoints
         ↓
Step 5: Frontend Types & API
         ↓
Step 6: Settings Page & Routing
         ↓
Step 7: Settings Components
         ↓
Step 8: Theme Sync & Integration
```

---

## Step 1: Extend User Model

### 1.1 Add Settings Fields

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

---

## Step 2: Create Migration

```bash
# Create migration file
cd phase2/backend
# Manually create: alembic/versions/20251215_000002_add_user_settings.py
```

```python
# Migration content
def upgrade() -> None:
    op.add_column('users', sa.Column('theme', sa.String(20), nullable=False, server_default='system'))
    op.add_column('users', sa.Column('email_notifications', sa.Boolean(), nullable=False, server_default='true'))

def downgrade() -> None:
    op.drop_column('users', 'email_notifications')
    op.drop_column('users', 'theme')
```

```bash
# Run migration
uv run alembic upgrade head
```

---

## Step 3: Add Backend Schemas

```python
# app/schemas/auth.py - ADD schemas

class UserSettingsUpdate(BaseModel):
    theme: str | None = Field(default=None, pattern="^(light|dark|system)$")
    email_notifications: bool | None = None

class UserSettingsResponse(BaseModel):
    theme: str
    email_notifications: bool
    model_config = {"from_attributes": True}

class PasswordChangeRequest(BaseModel):
    current_password: str = Field(..., min_length=1)
    new_password: str = Field(..., min_length=8)

class MessageResponse(BaseModel):
    message: str
```

### 3.1 Update UserResponse

```python
# app/schemas/auth.py - ADD to UserResponse
theme: str = Field(..., description="Theme preference")
email_notifications: bool = Field(..., description="Email notifications")
```

---

## Step 4: Add Backend Endpoints

### 4.1 Settings Update

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
```

### 4.2 Password Change

```python
@router.post("/change-password", response_model=MessageResponse)
@limiter.limit("5 per hour")
async def change_password(
    request: Request,
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
```

### 4.3 Account Deletion

```python
@router.delete("/me", status_code=status.HTTP_204_NO_CONTENT)
async def delete_account(
    current_user: CurrentUser,
    db: DbSession,
) -> None:
    """Delete current user's account and all data."""
    await auth_service.delete_user(db, current_user)
```

### 4.4 Service Functions

```python
# app/services/auth_service.py - ADD functions

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

---

## Step 5: Frontend Types & API

### 5.1 Settings Types

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
```

### 5.2 RTK Query Endpoints

```typescript
// src/services/settingsApi.ts
export const settingsApi = api.injectEndpoints({
  endpoints: (builder) => ({
    updateSettings: builder.mutation<UserSettings, UserSettingsUpdate>({
      query: (data) => ({ url: '/auth/me/settings', method: 'PATCH', body: data }),
      invalidatesTags: ['User'],
    }),
    changePassword: builder.mutation<{ message: string }, PasswordChangeRequest>({
      query: (data) => ({ url: '/auth/change-password', method: 'POST', body: data }),
    }),
    deleteAccount: builder.mutation<void, void>({
      query: () => ({ url: '/auth/me', method: 'DELETE' }),
    }),
  }),
});

export const { useUpdateSettingsMutation, useChangePasswordMutation, useDeleteAccountMutation } = settingsApi;
```

### 5.3 Zod Validation

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
```

---

## Step 6: Settings Page & Routing

### 6.1 Add Route

```typescript
// src/routes/index.tsx
SETTINGS: '/settings'
```

### 6.2 Wire Up in App.tsx

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

---

## Step 7: Settings Components

### 7.1 Install shadcn Switch

```bash
cd phase2/frontend && npx shadcn@latest add switch
```

### 7.2 SettingsPage Structure

```tsx
// src/pages/SettingsPage.tsx
<div className="space-y-8">
  <ThemeSettings />
  <NotificationSettings />
  <PasswordChangeForm />
  <DeleteAccountSection />
</div>
```

---

## Step 8: Theme Sync

### 8.1 Sync on Login

```typescript
// In login success handler or App.tsx useEffect
const { data: user } = useGetCurrentUserQuery();

useEffect(() => {
  if (user?.theme) {
    setTheme(user.theme as Theme);
  }
}, [user?.theme, setTheme]);
```

### 8.2 Sync on Change

```typescript
// In ThemeSettings.tsx
const handleThemeChange = async (newTheme: Theme) => {
  setTheme(newTheme); // Optimistic update
  await updateSettings({ theme: newTheme });
};
```

---

## Verification Checklist

- [ ] Migration adds theme and email_notifications columns
- [ ] `PATCH /auth/me/settings` updates settings
- [ ] `POST /auth/change-password` requires correct current password
- [ ] `DELETE /auth/me` removes user and tasks
- [ ] SettingsPage renders at `/settings`
- [ ] Theme changes persist across sessions
- [ ] Delete account dialog requires typing "DELETE"
- [ ] Toast notifications for all actions

---

## Quick Commands

```bash
# Backend - run migration
cd phase2/backend && uv run alembic upgrade head

# Backend - run server
cd phase2/backend && uv run uvicorn app.main:app --reload

# Frontend - install switch component
cd phase2/frontend && npx shadcn@latest add switch

# Frontend - run dev server
cd phase2/frontend && npm run dev

# Test settings endpoint
curl -X PATCH http://localhost:8000/api/auth/me/settings \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"theme": "dark"}'

# Test password change
curl -X POST http://localhost:8000/api/auth/change-password \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"current_password": "old", "new_password": "newpassword"}'
```
