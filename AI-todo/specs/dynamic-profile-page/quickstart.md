# Quickstart — Dynamic Profile Page

**Date**: 2025-12-15
**Feature**: dynamic-profile-page

## Prerequisites

- Node.js 18+
- Python 3.11+
- Running backend (`uvicorn app.main:app`)
- Running frontend (`npm run dev`)
- Authenticated user (JWT token)

## Implementation Order

```
Step 1: Backend PATCH endpoint
   ↓
Step 2: Frontend types + API
   ↓
Step 3: Profile page + routing
   ↓
Step 4: Profile components
   ↓
Step 5: Integration + testing
```

---

## Step 1: Backend — PATCH /auth/me

### 1.1 Add UserUpdate Schema

```python
# app/schemas/auth.py
class UserUpdate(BaseModel):
    """Schema for profile update request."""
    first_name: str | None = Field(default=None, max_length=100)
    last_name: str | None = Field(default=None, max_length=100)
    avatar_url: str | None = Field(default=None)
```

### 1.2 Add Service Function

```python
# app/services/auth_service.py
async def update_user_profile(
    db: AsyncSession,
    user: User,
    data: UserUpdate,
) -> User:
    """Update user profile fields."""
    update_data = data.model_dump(exclude_unset=True)
    for field, value in update_data.items():
        setattr(user, field, value)
    db.add(user)
    await db.commit()
    await db.refresh(user)
    return user
```

### 1.3 Add Route

```python
# app/api/routes/auth.py
@router.patch("/me", response_model=UserResponse)
async def update_current_user_profile(
    data: UserUpdate,
    current_user: CurrentUser,
    db: DbSession,
) -> UserResponse:
    """Update current user's profile."""
    user = await auth_service.update_user_profile(db, current_user, data)
    return UserResponse.model_validate(user)
```

---

## Step 2: Frontend — Types + API

### 2.1 Add User Types

```typescript
// src/types/user.ts
export interface User {
  id: string;
  email: string;
  first_name: string | null;
  last_name: string | null;
  display_name: string;
  avatar_url: string | null;
  is_active: boolean;
  created_at: string;
}

export interface UserUpdateRequest {
  first_name?: string | null;
  last_name?: string | null;
  avatar_url?: string | null;
}

export interface TaskStats {
  total: number;
  completed: number;
  pending: number;
  completionRate: number;
}
```

### 2.2 Add RTK Query Endpoints

```typescript
// src/services/userApi.ts
import { api, TAG_TYPES } from './api';
import type { User, UserUpdateRequest } from '@/types/user';

export const userApi = api.injectEndpoints({
  endpoints: (builder) => ({
    getCurrentUser: builder.query<User, void>({
      query: () => '/auth/me',
      providesTags: [TAG_TYPES.User],
    }),
    updateProfile: builder.mutation<User, UserUpdateRequest>({
      query: (data) => ({
        url: '/auth/me',
        method: 'PATCH',
        body: data,
      }),
      invalidatesTags: [TAG_TYPES.User],
    }),
  }),
});

export const { useGetCurrentUserQuery, useUpdateProfileMutation } = userApi;
```

### 2.3 Add Zod Validation

```typescript
// src/lib/validations/profile.ts
import { z } from 'zod';

export const profileSchema = z.object({
  firstName: z.string().max(100).nullable().optional(),
  lastName: z.string().max(100).nullable().optional(),
});

export type ProfileFormData = z.infer<typeof profileSchema>;
```

---

## Step 3: Profile Page + Routing

### 3.1 Add Route

```typescript
// src/routes/index.tsx
export const ROUTES = {
  // ... existing
  PROFILE: '/profile',
} as const;
```

### 3.2 Add Route in App.tsx

```tsx
// App.tsx - add import
import { ProfilePage } from '@/pages/ProfilePage';

// Add route
<Route
  path={ROUTES.PROFILE}
  element={
    <ProtectedRoute>
      <ProfilePage />
    </ProtectedRoute>
  }
/>
```

### 3.3 Create ProfilePage

```tsx
// src/pages/ProfilePage.tsx
export function ProfilePage() {
  const { data: user, isLoading, error } = useGetCurrentUserQuery();

  if (isLoading) return <ProfileSkeleton />;
  if (error) return <ErrorState />;

  return (
    <div className="container py-8">
      <ProfileHeader user={user} />
      <TaskStatsCards />
      <ProfileForm user={user} />
    </div>
  );
}
```

---

## Step 4: Profile Components

### 4.1 Install shadcn Components

```bash
npx shadcn@latest add avatar dialog form
```

### 4.2 ProfileForm Component

```tsx
// src/components/profile/ProfileForm.tsx
export function ProfileForm({ user }: { user: User }) {
  const [updateProfile, { isLoading }] = useUpdateProfileMutation();

  const form = useForm<ProfileFormData>({
    resolver: zodResolver(profileSchema),
    defaultValues: {
      firstName: user.first_name,
      lastName: user.last_name,
    },
  });

  const onSubmit = async (data: ProfileFormData) => {
    try {
      await updateProfile({
        first_name: data.firstName,
        last_name: data.lastName,
      }).unwrap();
      toast.success('Profile updated!');
    } catch {
      toast.error('Failed to update profile');
    }
  };

  return (
    <Dialog>
      <DialogTrigger asChild>
        <Button>Edit Profile</Button>
      </DialogTrigger>
      <DialogContent>
        <Form {...form}>
          <form onSubmit={form.handleSubmit(onSubmit)}>
            {/* Form fields */}
          </form>
        </Form>
      </DialogContent>
    </Dialog>
  );
}
```

### 4.3 TaskStatsCards Component

```tsx
// src/components/profile/TaskStatsCards.tsx
export function TaskStatsCards() {
  const { data } = useGetTasksQuery({ limit: 1000 });

  const stats = useMemo(() => {
    if (!data?.items) return { total: 0, completed: 0, pending: 0, rate: 0 };
    const total = data.total;
    const completed = data.items.filter(t => t.status === 'completed').length;
    return {
      total,
      completed,
      pending: total - completed,
      rate: total ? Math.round((completed / total) * 100) : 0,
    };
  }, [data]);

  return (
    <div className="grid grid-cols-3 gap-4">
      <StatsCard title="Total" value={stats.total} icon={ListTodo} />
      <StatsCard title="Completed" value={stats.completed} icon={CheckCircle} />
      <StatsCard title="Pending" value={stats.pending} icon={Clock} />
    </div>
  );
}
```

---

## Step 5: Testing Checklist

### Backend Tests

```python
# tests/unit/test_profile_update.py
async def test_update_profile_success():
    # PATCH /auth/me with valid data returns 200

async def test_update_profile_unauthorized():
    # PATCH /auth/me without token returns 401

async def test_update_profile_invalid_data():
    # PATCH /auth/me with >100 char name returns 400
```

### Frontend Tests

```typescript
// ProfilePage.test.tsx
it('displays user profile data');
it('shows loading skeleton');
it('opens edit dialog on button click');
it('submits form and shows success toast');
it('displays task statistics');
```

---

## Quick Commands

```bash
# Backend - run server
cd phase2/backend && uvicorn app.main:app --reload

# Frontend - install new components
cd phase2/frontend && npx shadcn@latest add avatar dialog form

# Frontend - run dev server
cd phase2/frontend && npm run dev

# Run backend tests
cd phase2/backend && pytest tests/ -v

# Test PATCH endpoint manually
curl -X PATCH http://localhost:8000/api/auth/me \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"first_name": "Jane", "last_name": "Smith"}'
```

---

## Verification Checklist

- [ ] `GET /auth/me` returns user with avatar_url field
- [ ] `PATCH /auth/me` updates first_name, last_name
- [ ] ProfilePage loads at `/profile` route
- [ ] Edit dialog opens and pre-fills current values
- [ ] Form submission updates profile
- [ ] Task stats show correct counts
- [ ] Toast notifications work
- [ ] Loading states display correctly
