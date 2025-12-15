# Specification — Dynamic Profile Page

## Feature Overview
Implement a user profile page that displays real user data fetched from `/auth/me`, allows profile editing via `PATCH /users/me`, shows task statistics, and supports avatar upload using shadcn Form components.

## Functional Specifications

### 1. Profile Data Display
- Fetch authenticated user data from `GET /auth/me`
- Display fields:
  - **First Name** (editable)
  - **Last Name** (editable)
  - **Email** (read-only, displayed)
  - **Avatar** (editable via upload)
  - **Display Name** (computed, read-only)
  - **Member Since** (created_at, formatted)

### 2. Edit Profile Form
- shadcn Form with React Hook Form + Zod validation
- Editable fields:
  - `firstName`: optional, max 100 chars
  - `lastName`: optional, max 100 chars
- Submit via `PATCH /users/me` endpoint
- Show loading state during submission
- Toast notification on success/error
- Optimistic UI update with RTK Query cache invalidation

### 3. Task Statistics Cards
- Display stats from user's tasks:
  - **Total Tasks**: count of all tasks
  - **Completed Tasks**: count of tasks with status "completed"
  - **Pending Tasks**: count of tasks with status "pending"
  - **Completion Rate**: percentage (completed/total × 100)
- Use shadcn Card components with icons
- Skeleton loading state while fetching

### 4. Avatar Upload
- Avatar display with fallback (initials from display_name)
- Click-to-upload functionality
- Supported formats: JPEG, PNG, WebP
- Max file size: 2MB
- Preview before confirming upload
- Upload via `POST /users/me/avatar` (multipart/form-data)
- Cache busting for updated avatar URL

## Technical Requirements

### Backend Changes Required

#### New Endpoint: `PATCH /users/me`
```python
# Request body (UserUpdate schema)
{
  "first_name": "string | null",  # optional
  "last_name": "string | null"    # optional
}

# Response: UserResponse (existing schema)
```

#### New Endpoint: `POST /users/me/avatar`
```python
# Request: multipart/form-data with 'file' field
# Response:
{
  "avatar_url": "string"
}
```

#### User Model Update
- Add `avatar_url: str | None` field to User model
- Create Alembic migration

### Frontend Implementation

#### New Files
| File | Purpose |
|------|---------|
| `src/pages/ProfilePage.tsx` | Main profile page component |
| `src/components/profile/ProfileForm.tsx` | Edit profile form |
| `src/components/profile/AvatarUpload.tsx` | Avatar upload component |
| `src/components/profile/TaskStatsCards.tsx` | Task statistics display |
| `src/services/userApi.ts` | RTK Query user endpoints |
| `src/types/user.ts` | User type definitions |
| `src/lib/validations/profile.ts` | Zod schema for profile form |

#### Route Addition
```typescript
// routes/index.tsx
PROFILE: '/profile'
```

#### API Endpoints (RTK Query)
```typescript
// userApi.ts
- useGetCurrentUserQuery()     // GET /auth/me
- useUpdateProfileMutation()   // PATCH /users/me
- useUploadAvatarMutation()    // POST /users/me/avatar
- useGetTaskStatsQuery()       // GET /tasks/stats (or derive from existing)
```

## UI/UX Requirements

### Layout Structure
```
┌─────────────────────────────────────────┐
│  Profile Header                         │
│  ┌──────┐  Name + Email                │
│  │Avatar│  Member since: date          │
│  └──────┘  [Edit Profile]              │
├─────────────────────────────────────────┤
│  Task Statistics                        │
│  ┌────────┐ ┌────────┐ ┌────────┐      │
│  │ Total  │ │Complete│ │Pending │      │
│  │  12    │ │   8    │ │   4    │      │
│  └────────┘ └────────┘ └────────┘      │
├─────────────────────────────────────────┤
│  Edit Profile Form (collapsible/modal)  │
│  - First Name [input]                   │
│  - Last Name [input]                    │
│  [Cancel] [Save Changes]                │
└─────────────────────────────────────────┘
```

### States
- **Loading**: Skeleton placeholders for avatar, name, stats
- **Error**: Error message with retry button
- **Empty Stats**: Show "0" with encouraging message
- **Edit Mode**: Inline form or modal dialog

## Constraints
- Avatar stored as URL (external storage or base64 for MVP)
- Profile updates do not change email (email is immutable post-registration)
- Task stats derived from existing `/tasks` endpoint response
- Use existing shadcn/ui components (Card, Input, Button, Avatar, Skeleton)

## Acceptance Criteria
- [ ] Profile page accessible at `/profile` route (protected)
- [ ] Displays current user data from `GET /auth/me`
- [ ] Edit form validates with Zod (firstName/lastName max 100 chars)
- [ ] `PATCH /users/me` updates user profile successfully
- [ ] Task stats cards show accurate counts (total/completed/pending)
- [ ] Avatar upload shows preview before submission
- [ ] Loading skeletons display during data fetch
- [ ] Toast notifications for success/error states
- [ ] RTK Query cache invalidates on profile update
- [ ] Mobile responsive layout
- [ ] Accessible (proper labels, ARIA attributes)

## Out of Scope
- Password change (separate feature)
- Account deletion
- Email change/verification
- Social login connections
- Activity history/audit log

## Dependencies
- Existing: `GET /auth/me` endpoint (already implemented)
- Existing: RTK Query base API setup
- Existing: shadcn/ui component library
- Required: Backend `PATCH /users/me` endpoint
- Required: Backend `POST /users/me/avatar` endpoint (optional for MVP)
- Required: User model avatar_url field migration

## API Contract Reference

### GET /auth/me (existing)
```json
{
  "id": "uuid",
  "email": "user@example.com",
  "first_name": "John",
  "last_name": "Doe",
  "display_name": "John Doe",
  "is_active": true,
  "created_at": "2025-01-15T10:30:00Z"
}
```

### PATCH /users/me (new)
```json
// Request
{
  "first_name": "Jane",
  "last_name": "Smith"
}

// Response: Same as GET /auth/me
```

### Task Stats (derive from GET /tasks)
```json
{
  "total": 12,
  "completed": 8,
  "pending": 4,
  "completion_rate": 66.7
}
```
