# Research — Dynamic Settings Page

**Date**: 2025-12-15
**Feature**: dynamic-settings-page
**Status**: Complete

## Research Questions

### 1. User Settings Storage Strategy

**Question**: Should settings be stored in User model or separate UserSettings table?

**Decision**: Extend User model directly

**Rationale**:
- MVP simplicity - no joins needed for common operations
- Settings are always fetched with user profile
- Only 2 fields (theme, email_notifications)
- Can migrate to separate table later if needed

**Alternatives Considered**:
| Option | Pros | Cons |
|--------|------|------|
| Extend User model | Simple, no joins, single migration | User model grows |
| Separate UserSettings | Clean separation, extensible | Extra table, joins, FK management |
| JSON column | Flexible schema | Harder to query, validate |

---

### 2. Password Change Security

**Question**: What security measures are needed for password change?

**Decision**: Require current password + rate limiting

**Rationale**:
- Current password verification prevents unauthorized changes if session hijacked
- Rate limiting (5/hour) prevents brute force attempts on current password
- No need to invalidate other sessions for MVP (single session per user typical)

**Implementation**:
```python
# Rate limit in routes/auth.py
@limiter.limit("5 per hour")
async def change_password(...)
```

---

### 3. Account Deletion Strategy

**Question**: Soft delete vs hard delete for user accounts?

**Decision**: Hard delete with cascade

**Rationale**:
- GDPR compliance requires actual data deletion on request
- Simpler implementation (no "deleted" flags to check everywhere)
- Cascade delete handles related data (tasks)

**Implementation**:
```python
# SQLModel cascade delete
await db.delete(user)
await db.commit()  # Tasks deleted via FK cascade
```

---

### 4. Theme Sync Strategy

**Question**: How to sync theme between frontend Zustand store and backend?

**Decision**: Backend as source of truth, sync on login

**Rationale**:
- Backend persistence ensures theme follows user across devices
- localStorage as fallback for pre-login state
- On login: fetch settings → update Zustand store
- On theme change: update Zustand immediately (optimistic) → sync to backend

**Flow**:
```
Login Success → Fetch /auth/me → Extract theme → setTheme(user.theme)
                                                       ↓
                                              Zustand store updated
                                                       ↓
                                              useTheme hook applies
```

---

### 5. RTK Query Cache Strategy

**Question**: How to handle settings cache with existing User cache?

**Decision**: Reuse User tag, separate settings endpoint

**Rationale**:
- Settings are part of user data but have separate update endpoint
- `PATCH /auth/me/settings` invalidates `User` tag
- Frontend can derive settings from user data or use dedicated endpoint

**Implementation**:
```typescript
updateSettings: builder.mutation({
  query: (data) => ({ url: '/auth/me/settings', method: 'PATCH', body: data }),
  invalidatesTags: ['User'],
}),
```

---

### 6. Form Validation Patterns

**Question**: What Zod schemas are needed for settings forms?

**Decision**: Separate schemas for password and settings

**Schemas**:
```typescript
// Password change
export const passwordChangeSchema = z.object({
  currentPassword: z.string().min(1, 'Current password is required'),
  newPassword: z.string().min(8, 'Password must be at least 8 characters'),
  confirmPassword: z.string().min(1, 'Please confirm your password'),
}).refine(data => data.newPassword === data.confirmPassword, {
  message: 'Passwords do not match',
  path: ['confirmPassword'],
});

// Settings (no validation needed - toggle/select only)
```

---

## Technology Decisions Summary

| Component | Technology | Justification |
|-----------|------------|---------------|
| Form | React Hook Form + Zod | Existing pattern in auth forms |
| State | RTK Query + Zustand | RTK for API, Zustand for theme |
| UI | shadcn (Switch, Dialog, Input) | Project standard |
| Security | Argon2id + rate limiting | Existing security stack |
| Deletion | Hard delete + cascade | GDPR compliance |

## Dependencies Verified

| Dependency | Status | Notes |
|------------|--------|-------|
| User model | ✅ Exists | Need to extend with theme, email_notifications |
| Auth routes | ✅ Exists | Add new endpoints |
| RTK Query | ✅ Setup | Add settingsApi.ts |
| shadcn Switch | ❌ Need | `npx shadcn@latest add switch` |
| Zustand theme | ✅ Exists | Already synced to localStorage |

## Open Items

None - all research questions resolved.

## Next Steps

1. Create `data-model.md` with User model extension
2. Generate API contracts for new endpoints
3. Proceed to `plan.md` with 8 implementation steps
