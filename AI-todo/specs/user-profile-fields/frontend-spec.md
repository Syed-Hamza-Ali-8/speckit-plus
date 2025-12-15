# Specification — Frontend Register Form (firstName/lastName)

## Feature Overview

Add `firstName` and `lastName` input fields to the RegisterForm component, update Zod validation schema, update RTK mutation types, and show a personalized welcome toast.

## Current State

### RegisterForm.tsx
- Fields: email, password, confirmPassword
- Uses react-hook-form + Zod validation
- RTK Query mutation for registration

### Zod Schema (`lib/validations/auth.ts`)
```typescript
registerSchema = z.object({
  email: z.string().min(1).email(),
  password: z.string().min(8),
  confirmPassword: z.string().min(1),
}).refine(...)
```

### Types (`types/auth.ts`)
```typescript
interface RegisterRequest {
  email: string;
  password: string;
}
```

---

## Functional Specifications

### 1. Update Types (`types/auth.ts`)

```typescript
export interface RegisterRequest {
  email: string;
  password: string;
  first_name?: string;  // Optional - matches backend snake_case
  last_name?: string;   // Optional - matches backend snake_case
}

export interface JWTPayload {
  sub: string;
  email: string;
  name: string;  // NEW - display_name from backend
  exp: number;
  iat: number;
}
```

### 2. Update Zod Schema (`lib/validations/auth.ts`)

```typescript
export const registerSchema = z
  .object({
    firstName: z
      .string()
      .max(100, 'First name must be less than 100 characters')
      .optional()
      .transform(v => v?.trim() || undefined),
    lastName: z
      .string()
      .max(100, 'Last name must be less than 100 characters')
      .optional()
      .transform(v => v?.trim() || undefined),
    email: z.string().min(1, 'Email is required').email('Invalid email'),
    password: z.string().min(1, 'Password is required').min(8, 'Min 8 characters'),
    confirmPassword: z.string().min(1, 'Please confirm your password'),
  })
  .refine((data) => data.password === data.confirmPassword, {
    message: 'Passwords do not match',
    path: ['confirmPassword'],
  });
```

### 3. Update RegisterForm.tsx

**New Fields (before email):**
```tsx
{/* First Name Field */}
<div className="space-y-2">
  <Label htmlFor="firstName">First Name (optional)</Label>
  <Input
    id="firstName"
    type="text"
    placeholder="John"
    autoComplete="given-name"
    disabled={isLoading}
    {...registerField('firstName')}
  />
  {errors.firstName && <p className="text-sm text-destructive">{errors.firstName.message}</p>}
</div>

{/* Last Name Field */}
<div className="space-y-2">
  <Label htmlFor="lastName">Last Name (optional)</Label>
  <Input
    id="lastName"
    type="text"
    placeholder="Doe"
    autoComplete="family-name"
    disabled={isLoading}
    {...registerField('lastName')}
  />
  {errors.lastName && <p className="text-sm text-destructive">{errors.lastName.message}</p>}
</div>
```

**Updated Submit Handler:**
```typescript
const onSubmit = async (data: RegisterFormData) => {
  try {
    const result = await register({
      email: data.email,
      password: data.password,
      first_name: data.firstName || undefined,  // Convert to snake_case
      last_name: data.lastName || undefined,
    }).unwrap();

    localStorage.setItem('token', result.access_token);

    // Personalized welcome toast
    const displayName = data.firstName || data.email.split('@')[0];
    toast.success(`Welcome, ${displayName}! Your account has been created.`);

    onSuccess?.();
  } catch (error) {
    // ... existing error handling
  }
};
```

---

## UI Layout

```
┌─────────────────────────────────────┐
│  First Name (optional)              │
│  ┌─────────────────────────────┐    │
│  │ John                        │    │
│  └─────────────────────────────┘    │
│                                     │
│  Last Name (optional)               │
│  ┌─────────────────────────────┐    │
│  │ Doe                         │    │
│  └─────────────────────────────┘    │
│                                     │
│  Email *                            │
│  ┌─────────────────────────────┐    │
│  │ user@example.com            │    │
│  └─────────────────────────────┘    │
│                                     │
│  Password *                         │
│  ┌─────────────────────────────┐👁  │
│  │ ••••••••                    │    │
│  └─────────────────────────────┘    │
│                                     │
│  Confirm Password *                 │
│  ┌─────────────────────────────┐👁  │
│  │ ••••••••                    │    │
│  └─────────────────────────────┘    │
│                                     │
│  ┌─────────────────────────────┐    │
│  │      Create Account         │    │
│  └─────────────────────────────┘    │
└─────────────────────────────────────┘
```

---

## Acceptance Criteria

- [ ] First name and last name fields appear before email
- [ ] Fields are optional (no validation error when empty)
- [ ] Max length 100 characters enforced
- [ ] Whitespace-only values treated as empty
- [ ] API receives `first_name` / `last_name` (snake_case)
- [ ] Welcome toast shows first name if provided, email prefix otherwise
- [ ] Existing validation (email, password match) still works
- [ ] Form disabled during submission

---

## Files to Modify

| File | Changes |
|------|---------|
| `types/auth.ts` | Add `first_name`, `last_name` to `RegisterRequest`, `name` to `JWTPayload` |
| `lib/validations/auth.ts` | Add `firstName`, `lastName` to `registerSchema` |
| `components/auth/RegisterForm.tsx` | Add input fields, update submit handler |
