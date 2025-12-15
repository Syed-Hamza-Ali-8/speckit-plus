# Research — Dynamic Profile Page

**Date**: 2025-12-15
**Feature**: dynamic-profile-page
**Status**: Complete

## Research Questions

### 1. Avatar Storage Strategy

**Question**: How should avatar images be stored for MVP?

**Decision**: Base64 data URL storage in database

**Rationale**:
- Simplest implementation for MVP with no external dependencies
- No need for S3/cloud storage configuration
- Avatar images are small (max 2MB, compressed to ~50KB)
- Easy to serve directly from UserResponse

**Alternatives Considered**:
| Option | Pros | Cons |
|--------|------|------|
| S3/Cloud Storage | Scalable, CDN-ready | Requires AWS setup, additional cost |
| Local File System | Simple | Not portable, won't work in containers |
| Base64 in DB | Zero dependencies, portable | Slightly larger DB size |

**Implementation**: Store as `avatar_url` field with `data:image/jpeg;base64,...` format

---

### 2. Task Statistics Derivation

**Question**: Should task stats come from a dedicated endpoint or be derived from existing `/tasks`?

**Decision**: Derive from existing `/tasks` endpoint in frontend

**Rationale**:
- Tasks are already fetched for TasksPage
- RTK Query caching means no additional API calls
- Simple count/filter operations in JavaScript
- No backend changes required for MVP

**Alternatives Considered**:
| Option | Pros | Cons |
|--------|------|------|
| `GET /tasks/stats` endpoint | Server-side calculation, accurate | New endpoint, more backend work |
| Derive from `/tasks` | Reuses existing data | Limited to loaded tasks |
| Store stats in User model | Fast reads | Stale data, sync complexity |

**Implementation**: Use RTK Query selector to compute stats from cached tasks

---

### 3. Profile Edit UI Pattern

**Question**: Should profile editing use inline form, modal dialog, or separate page?

**Decision**: Modal dialog (shadcn Dialog component)

**Rationale**:
- Clean separation between view and edit modes
- Prevents accidental edits
- Consistent with shadcn patterns
- Mobile-friendly overlay

**Alternatives Considered**:
| Option | Pros | Cons |
|--------|------|------|
| Inline Form | Quick edits | Complex state management |
| Modal Dialog | Clean UX, focused editing | Extra click to edit |
| Separate Page | Simple routing | Overkill for 2 fields |

---

### 4. RTK Query Cache Invalidation Strategy

**Question**: How to invalidate user data after profile update?

**Decision**: Use RTK Query tags with automatic invalidation

**Rationale**:
- Consistent with existing task caching pattern
- `User` tag already defined in `api.ts`
- Mutation automatically refetches on invalidation

**Implementation**:
```typescript
// userApi.ts
updateProfile: builder.mutation({
  query: (data) => ({ url: '/auth/me', method: 'PATCH', body: data }),
  invalidatesTags: ['User'],
}),
getCurrentUser: builder.query({
  query: () => '/auth/me',
  providesTags: ['User'],
}),
```

---

### 5. Form Validation Pattern

**Question**: What Zod schema patterns to use for profile form?

**Decision**: Match existing `auth.ts` validation patterns

**Rationale**:
- Consistency with RegisterForm validation
- Same field constraints (max 100 chars)
- Reuse existing error message styles

**Schema Design**:
```typescript
export const profileSchema = z.object({
  firstName: z.string().max(100).optional().nullable(),
  lastName: z.string().max(100).optional().nullable(),
});
```

---

## Technology Decisions Summary

| Component | Technology | Justification |
|-----------|------------|---------------|
| Form | React Hook Form + Zod | Existing pattern in auth forms |
| State | RTK Query | Existing API layer |
| UI | shadcn (Dialog, Card, Avatar, Input) | Project standard |
| Avatar Storage | Base64 in DB | MVP simplicity |
| File Upload | Native input + FileReader | No external dependencies |

## Dependencies Verified

| Dependency | Status | Notes |
|------------|--------|-------|
| `GET /auth/me` | ✅ Exists | Returns UserResponse |
| `PATCH /auth/me` | ❌ Needed | New endpoint for profile update |
| RTK Query `User` tag | ✅ Defined | In `api.ts` TAG_TYPES |
| shadcn Avatar | ❌ Not installed | Need to add via CLI |
| shadcn Form | ❌ Not installed | Need to add via CLI |

## Open Items

None - all research questions resolved.

## Next Steps

1. Create `data-model.md` with UserUpdate schema
2. Generate API contracts for `PATCH /auth/me`
3. Proceed to `quickstart.md` for implementation guide
