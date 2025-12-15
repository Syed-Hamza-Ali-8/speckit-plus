# Specification — Dynamic Settings Page

## Feature Overview
Implement a user settings page with theme preferences synced to backend, secure password change functionality, email notification preferences, and account deletion capability with confirmation dialog.

## Functional Specifications

### 1. Theme Settings (Persisted to Backend)
- Display current theme preference (light/dark/system)
- Theme toggle synced to `PATCH /auth/me/settings`
- On login, fetch user settings and apply theme
- Fallback to localStorage if user settings unavailable
- Real-time preview when changing theme

### 2. Password Change Form
- Current password field (required for verification)
- New password field (min 8 characters)
- Confirm new password field (must match)
- Submit via `POST /auth/change-password`
- Show/hide password toggle for all fields
- Success: Show toast, clear form
- Error: Display specific error message (wrong current password, etc.)

### 3. Email Notifications Toggle
- Toggle for email notifications (on/off)
- Persisted via `PATCH /auth/me/settings`
- Categories (future expansion):
  - Task reminders
  - Weekly digest
  - Account alerts

### 4. Delete Account
- "Delete Account" button with danger styling
- Confirmation dialog with:
  - Warning message about permanent deletion
  - Input to type "DELETE" to confirm
  - Cancel and Confirm buttons
- Submit via `DELETE /auth/me`
- On success: Clear tokens, redirect to landing page
- Show error if deletion fails

## Technical Requirements

### Backend Changes Required

#### New Model: UserSettings (or extend User)

**Option A: Extend User Model**
```python
# Add to User model
theme: str = Field(default="system", max_length=20)  # light/dark/system
email_notifications: bool = Field(default=True)
```

**Option B: Separate UserSettings Table**
```python
class UserSettings(SQLModel, table=True):
    user_id: UUID = Field(foreign_key="users.id", primary_key=True)
    theme: str = Field(default="system")
    email_notifications: bool = Field(default=True)
```

**Decision**: Extend User model (simpler for MVP, no joins needed)

#### New Endpoint: `PATCH /auth/me/settings`
```python
# Request body
{
  "theme": "dark",              # optional: "light" | "dark" | "system"
  "email_notifications": true   # optional: boolean
}

# Response: UserSettingsResponse
{
  "theme": "dark",
  "email_notifications": true
}
```

#### New Endpoint: `POST /auth/change-password`
```python
# Request body
{
  "current_password": "oldpass123",
  "new_password": "newpass456"
}

# Response: 200 OK
{
  "message": "Password changed successfully"
}

# Error responses:
# 401: Current password is incorrect
# 422: New password validation failed
```

#### New Endpoint: `DELETE /auth/me`
```python
# Request: No body required (authenticated via JWT)

# Response: 204 No Content

# Error responses:
# 401: Not authenticated
```

#### Schema Updates
```python
# app/schemas/auth.py

class UserSettingsUpdate(BaseModel):
    theme: str | None = Field(default=None, pattern="^(light|dark|system)$")
    email_notifications: bool | None = None

class UserSettingsResponse(BaseModel):
    theme: str
    email_notifications: bool

class PasswordChangeRequest(BaseModel):
    current_password: str = Field(..., min_length=1)
    new_password: str = Field(..., min_length=8)
```

### Frontend Implementation

#### New Files
| File | Purpose |
|------|---------|
| `src/pages/SettingsPage.tsx` | Main settings page component |
| `src/components/settings/ThemeSettings.tsx` | Theme toggle section |
| `src/components/settings/PasswordChangeForm.tsx` | Password change form |
| `src/components/settings/NotificationSettings.tsx` | Email notification toggles |
| `src/components/settings/DeleteAccountDialog.tsx` | Account deletion confirmation |
| `src/services/settingsApi.ts` | RTK Query settings endpoints |
| `src/types/settings.ts` | Settings type definitions |
| `src/lib/validations/settings.ts` | Zod schemas for forms |

#### Route Addition
```typescript
// routes/index.tsx
SETTINGS: '/settings'
```

#### API Endpoints (RTK Query)
```typescript
// settingsApi.ts
- useGetUserSettingsQuery()      // GET /auth/me (extract settings)
- useUpdateSettingsMutation()    // PATCH /auth/me/settings
- useChangePasswordMutation()    // POST /auth/change-password
- useDeleteAccountMutation()     // DELETE /auth/me
```

## UI/UX Requirements

### Layout Structure
```
┌─────────────────────────────────────────┐
│  Settings                               │
│  Manage your account preferences        │
├─────────────────────────────────────────┤
│                                         │
│  Appearance                             │
│  ┌─────────────────────────────────┐   │
│  │ Theme                           │   │
│  │ [Light] [Dark] [System]         │   │
│  └─────────────────────────────────┘   │
│                                         │
│  Notifications                          │
│  ┌─────────────────────────────────┐   │
│  │ Email Notifications    [Toggle] │   │
│  │ Receive updates about your tasks│   │
│  └─────────────────────────────────┘   │
│                                         │
│  Security                               │
│  ┌─────────────────────────────────┐   │
│  │ Change Password                 │   │
│  │ Current Password [........]     │   │
│  │ New Password     [........]     │   │
│  │ Confirm Password [........]     │   │
│  │ [Change Password]               │   │
│  └─────────────────────────────────┘   │
│                                         │
│  Danger Zone                            │
│  ┌─────────────────────────────────┐   │
│  │ Delete Account                  │   │
│  │ Permanently delete your account │   │
│  │ [Delete Account] (red button)   │   │
│  └─────────────────────────────────┘   │
│                                         │
└─────────────────────────────────────────┘
```

### States
- **Loading**: Skeleton placeholders for settings
- **Saving**: Disable controls, show spinner
- **Success**: Toast notification
- **Error**: Inline error message or toast
- **Delete Confirmation**: Modal with typed confirmation

### Delete Account Dialog
```
┌────────────────────────────────────────┐
│  Delete Account                    [X] │
├────────────────────────────────────────┤
│                                        │
│  ⚠️ This action cannot be undone.      │
│                                        │
│  This will permanently delete your     │
│  account and all associated data:      │
│  • All your tasks                      │
│  • Your profile information            │
│  • Your settings                       │
│                                        │
│  Type DELETE to confirm:               │
│  [________________]                    │
│                                        │
│  [Cancel]  [Delete Account] (disabled) │
│                                        │
└────────────────────────────────────────┘
```

## Constraints
- Theme changes apply immediately (optimistic UI)
- Password change requires current password verification
- Account deletion is irreversible
- Rate limit password change attempts (prevent brute force)
- Use existing shadcn/ui components (Switch, Input, Button, Dialog)

## Acceptance Criteria
- [ ] Settings page accessible at `/settings` route (protected)
- [ ] Theme toggle persists to backend and applies immediately
- [ ] On login, user's theme preference is loaded and applied
- [ ] Password change validates current password
- [ ] Password change enforces minimum 8 characters
- [ ] Password change shows success/error feedback
- [ ] Email notifications toggle persists to backend
- [ ] Delete account shows confirmation dialog
- [ ] Delete account requires typing "DELETE" to confirm
- [ ] Delete account clears session and redirects to home
- [ ] All forms show loading states during submission
- [ ] Toast notifications for success/error states
- [ ] Mobile responsive layout
- [ ] Accessible (proper labels, ARIA attributes)

## Out of Scope
- Two-factor authentication
- OAuth/social login management
- Session management (view active sessions)
- Export user data
- Email change functionality
- Notification categories (single toggle for MVP)

## Dependencies
- Existing: User model and authentication system
- Existing: RTK Query base API setup
- Existing: shadcn/ui component library
- Existing: Zustand theme store (will sync with backend)
- Required: Backend settings endpoints
- Required: User model extension (theme, email_notifications fields)
- Required: Alembic migration for new fields

## API Contract Reference

### PATCH /auth/me/settings
```json
// Request
{
  "theme": "dark",
  "email_notifications": false
}

// Response
{
  "theme": "dark",
  "email_notifications": false
}
```

### POST /auth/change-password
```json
// Request
{
  "current_password": "oldpassword123",
  "new_password": "newpassword456"
}

// Response (200)
{
  "message": "Password changed successfully"
}

// Error Response (401)
{
  "detail": "Current password is incorrect"
}
```

### DELETE /auth/me
```json
// Response: 204 No Content

// Error Response (401)
{
  "detail": "Not authenticated"
}
```

## Security Considerations

1. **Password Change**
   - Verify current password before allowing change
   - Rate limit to 5 attempts per hour
   - Log password change events
   - Invalidate other sessions after password change (optional)

2. **Account Deletion**
   - Soft delete or hard delete? (Decision: Hard delete for GDPR compliance)
   - Cascade delete all user data (tasks, settings)
   - Log deletion event before removing data

3. **Settings Access**
   - All settings endpoints require authentication
   - Users can only modify their own settings
