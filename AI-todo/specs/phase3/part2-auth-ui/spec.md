# Specification: Phase 3 Part 2 - Authentication UI

**Feature**: Production-ready authentication forms with validation, toast notifications, and smart redirects
**Date**: 2025-12-15
**Prerequisites**: Phase 3 Part 1 (Frontend Setup) complete
**Output**: Fully functional login/register forms integrated with backend API

---

## 1. Requirements

### 1.1 Form Validation

| Requirement | Description |
|-------------|-------------|
| **Library** | React Hook Form + Zod schema validation |
| **Email validation** | Valid email format, required field |
| **Password validation** | Minimum 8 characters, required field |
| **Confirm password** | Must match password field (register only) |
| **Real-time validation** | Show errors on blur and on submit |
| **Error display** | Inline error messages below each field |

### 1.2 Toast Notifications

| Event | Toast Type | Message | Auto-dismiss |
|-------|------------|---------|--------------|
| Login success | Success | "Welcome back!" | 3 seconds |
| Login failure (401) | Error | "Invalid email or password" | 5 seconds |
| Registration success | Success | "Account created successfully!" | 3 seconds |
| Registration failure (409) | Error | "Email already in use" | 5 seconds |
| Network error | Error | "Connection error. Please try again." | 5 seconds |
| Validation error | Error | "Please fix the errors above" | 5 seconds |

### 1.3 Loading States

| State | UI Behavior |
|-------|-------------|
| Form submitting | Button disabled, spinner + "Logging in..." / "Creating account..." |
| Button idle | Normal button text |
| Form disabled | All inputs disabled during submission |

### 1.4 Responsive Design

| Breakpoint | Layout |
|------------|--------|
| Mobile (<640px) | Full-width card with padding |
| Tablet (640-1024px) | Centered card, max-width 400px |
| Desktop (>1024px) | Centered card, max-width 400px |

### 1.5 Redirect Behavior

| Scenario | Behavior |
|----------|----------|
| Login success | Immediate redirect to return URL (if present) or `/tasks` |
| Register success | Auto-login after registration, redirect to `/tasks` |
| Already authenticated | Redirect away from auth pages to `/tasks` |
| Return URL support | Store `?returnTo=/path` in URL, use after login |

### 1.6 Password Field UX

| Feature | Behavior |
|---------|----------|
| Visibility toggle | Eye icon button to show/hide password |
| Default state | Password masked (dots) |
| Toggle behavior | Click eye icon → toggle between masked/visible |

### 1.7 Mobile UX

| Feature | Behavior |
|---------|----------|
| Keyboard handling | Auto-scroll form into view when keyboard appears |
| Input focus | Smooth scroll to focused field |

---

## 2. Components

### 2.1 LoginForm Component

**File**: `src/components/auth/LoginForm.tsx`

**Props**:
```typescript
interface LoginFormProps {
  onSuccess?: () => void;
  returnTo?: string;
}
```

**Zod Schema**:
```typescript
const loginSchema = z.object({
  email: z.string()
    .min(1, 'Email is required')
    .email('Invalid email address'),
  password: z.string()
    .min(1, 'Password is required')
    .min(8, 'Password must be at least 8 characters'),
});

type LoginFormData = z.infer<typeof loginSchema>;
```

**Features**:
- React Hook Form with Zod resolver
- Controlled inputs with shadcn/ui Input component
- Inline validation errors
- Submit button with loading spinner
- Link to register page
- Integration with `useLoginMutation` from RTK Query

### 2.2 RegisterForm Component

**File**: `src/components/auth/RegisterForm.tsx`

**Props**:
```typescript
interface RegisterFormProps {
  onSuccess?: () => void;
}
```

**Zod Schema**:
```typescript
const registerSchema = z.object({
  email: z.string()
    .min(1, 'Email is required')
    .email('Invalid email address'),
  password: z.string()
    .min(1, 'Password is required')
    .min(8, 'Password must be at least 8 characters'),
  confirmPassword: z.string()
    .min(1, 'Please confirm your password'),
}).refine((data) => data.password === data.confirmPassword, {
  message: 'Passwords do not match',
  path: ['confirmPassword'],
});

type RegisterFormData = z.infer<typeof registerSchema>;
```

**Features**:
- React Hook Form with Zod resolver
- Password confirmation with cross-field validation
- Inline validation errors
- Submit button with loading spinner
- Link to login page
- Integration with `useRegisterMutation` from RTK Query

### 2.3 LoadingButton Component

**File**: `src/components/ui/loading-button.tsx`

**Props**:
```typescript
interface LoadingButtonProps extends ButtonProps {
  loading?: boolean;
  loadingText?: string;
}
```

**Features**:
- Extends shadcn/ui Button
- Shows spinner icon when loading
- Displays custom loading text
- Disables button during loading

### 2.4 FormField Component

**File**: Uses shadcn/ui Form components

**Features**:
- Label with required indicator
- Input with error styling
- Error message display
- Accessible form controls

### 2.5 Updated Pages

**LoginPage** (`src/pages/LoginPage.tsx`):
- Extract return URL from query params
- Render LoginForm component
- Handle redirect after success

**RegisterPage** (`src/pages/RegisterPage.tsx`):
- Render RegisterForm component
- Handle redirect after success

---

## 3. Flow

### 3.1 Login Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                        LOGIN FLOW                                │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  1. User visits /login?returnTo=/tasks/123                      │
│                    │                                             │
│                    ▼                                             │
│  2. LoginPage extracts returnTo from URL                        │
│                    │                                             │
│                    ▼                                             │
│  3. User fills form → Zod validates on blur                     │
│                    │                                             │
│                    ▼                                             │
│  4. User clicks "Login" → Form validates all fields             │
│                    │                                             │
│          ┌────────┴────────┐                                    │
│          │                 │                                     │
│          ▼                 ▼                                     │
│   Validation FAIL    Validation PASS                            │
│          │                 │                                     │
│          ▼                 ▼                                     │
│   Show inline         5. useLoginMutation()                     │
│   errors                   │                                     │
│                           │                                     │
│                  ┌────────┴────────┐                            │
│                  │                 │                             │
│                  ▼                 ▼                             │
│           API SUCCESS        API FAILURE                        │
│                  │                 │                             │
│                  ▼                 ▼                             │
│   6. Store token         6. Show error toast                    │
│   7. Success toast       7. Keep form enabled                   │
│   8. Navigate to                                                │
│      returnTo or /tasks                                         │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2 Register Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                      REGISTER FLOW                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  1. User visits /register                                       │
│                    │                                             │
│                    ▼                                             │
│  2. User fills form → Zod validates on blur                     │
│     - Email validated                                           │
│     - Password validated (min 8 chars)                          │
│     - Confirm password validated (must match)                   │
│                    │                                             │
│                    ▼                                             │
│  3. User clicks "Register" → Form validates all fields          │
│                    │                                             │
│          ┌────────┴────────┐                                    │
│          │                 │                                     │
│          ▼                 ▼                                     │
│   Validation FAIL    Validation PASS                            │
│          │                 │                                     │
│          ▼                 ▼                                     │
│   Show inline         4. useRegisterMutation()                  │
│   errors                   │                                     │
│                           │                                     │
│                  ┌────────┴────────┐                            │
│                  │                 │                             │
│                  ▼                 ▼                             │
│           API SUCCESS        API FAILURE                        │
│                  │                 │                             │
│                  ▼                 ▼                             │
│   5. Store token         5. Show error toast                    │
│   6. Success toast          (email in use, etc.)                │
│   7. Navigate to /tasks  6. Keep form enabled                   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 3.3 Protected Route Flow (with Return URL)

```
┌─────────────────────────────────────────────────────────────────┐
│                   PROTECTED ROUTE FLOW                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  1. User visits /tasks (not authenticated)                      │
│                    │                                             │
│                    ▼                                             │
│  2. ProtectedRoute checks isAuthenticated                       │
│                    │                                             │
│                    ▼                                             │
│  3. Redirect to /login?returnTo=/tasks                          │
│                    │                                             │
│                    ▼                                             │
│  4. User logs in successfully                                   │
│                    │                                             │
│                    ▼                                             │
│  5. Redirect back to /tasks                                     │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 4. Success Criteria

### 4.1 Functional Requirements

| # | Criterion | Verification |
|---|-----------|--------------|
| 1 | Login form renders with email and password fields | Visit `/login`, form displays |
| 2 | Register form renders with email, password, confirm fields | Visit `/register`, form displays |
| 3 | Zod validation shows inline errors on invalid input | Enter invalid email, blur → error shows |
| 4 | Password minimum 8 characters enforced | Enter 7 chars → error shows |
| 5 | Confirm password must match | Enter mismatched passwords → error shows |
| 6 | Login success stores JWT and redirects | Login with valid creds → token in localStorage, redirect to `/tasks` |
| 7 | Login failure shows toast notification | Login with invalid creds → Sonner error toast |
| 8 | Register success stores JWT and redirects | Register → token in localStorage, redirect to `/tasks` |
| 9 | Register failure shows toast (email in use) | Register with existing email → error toast |
| 10 | Return URL redirect works | Visit `/tasks` → redirect to `/login?returnTo=/tasks` → login → back to `/tasks` |

### 4.2 UI/UX Requirements

| # | Criterion | Verification |
|---|-----------|--------------|
| 11 | Loading spinner shows during submission | Click login → button shows spinner |
| 12 | Form inputs disabled during submission | Click login → inputs disabled |
| 13 | Toast notifications use Sonner | Success/error toasts appear with animation |
| 14 | Responsive layout works on mobile | Resize to 375px → card fills width with padding |
| 15 | Dark mode styles apply to forms | Toggle dark mode → form adapts |

### 4.3 Technical Requirements

| # | Criterion | Verification |
|---|-----------|--------------|
| 16 | TypeScript compiles with no errors | `npx tsc --noEmit` passes |
| 17 | Production build succeeds | `npm run build` completes |
| 18 | React Hook Form integrated | Form uses `useForm` hook |
| 19 | Zod schemas defined | Validation schemas in form components |
| 20 | RTK Query mutations used | `useLoginMutation`, `useRegisterMutation` called |

---

## 5. Dependencies

### 5.1 New Packages

```bash
npm install react-hook-form @hookform/resolvers zod sonner
```

| Package | Version | Purpose |
|---------|---------|---------|
| `react-hook-form` | ^7.x | Form state management |
| `@hookform/resolvers` | ^3.x | Zod integration for RHF |
| `zod` | ^3.x | Schema validation |
| `sonner` | ^1.x | Toast notifications |

### 5.2 Existing Dependencies (from Part 1)

- `@reduxjs/toolkit` - RTK Query mutations
- `react-redux` - Redux provider
- `react-router-dom` - Routing and navigation
- `lucide-react` - Spinner icon
- shadcn/ui components - Button, Input, Label, Card

---

## 6. File Structure

```
src/
├── components/
│   ├── auth/
│   │   ├── LoginForm.tsx        # NEW
│   │   └── RegisterForm.tsx     # NEW
│   └── ui/
│       └── loading-button.tsx   # NEW (or enhance Button)
├── lib/
│   └── validations/
│       └── auth.ts              # NEW - Zod schemas
├── pages/
│   ├── LoginPage.tsx            # UPDATED - Use LoginForm
│   └── RegisterPage.tsx         # UPDATED - Use RegisterForm
├── routes/
│   └── ProtectedRoute.tsx       # UPDATED - Add returnTo support
└── App.tsx                      # UPDATED - Add Sonner Toaster
```

---

## 7. Related Documents

- [Phase 3 Part 1 spec.md](../part1-frontend-setup/spec.md) - Frontend setup
- [Phase 2 Part 2 spec.md](../../phase2/part2-authentication/spec.md) - Backend auth API
- [data-model.md](../part1-frontend-setup/data-model.md) - Auth types

---

## 8. Out of Scope

- Password reset / forgot password flow
- Email verification
- Social login (OAuth)
- Remember me checkbox
- Two-factor authentication
- Session timeout handling

---

## 9. Clarification Answers (from /sp.clarify)

| # | Question | Answer |
|---|----------|--------|
| Q1 | Form validation errors display | **A**: Inline field errors (below each invalid field) |
| Q2 | Password visibility toggle | **A**: Yes - Add eye icon toggle to show/hide password |
| Q3 | Toast auto-dismiss timing | **A**: 3 seconds for success, 5 seconds for errors |
| Q4 | Login success flow | **A**: Immediate redirect after login |
| Q5 | Register success behavior | **A**: Auto-login after registration, redirect to /tasks |
| Q6 | Network error messages | **A**: Generic "Connection error. Please try again." |
| Q7 | Mobile keyboard behavior | **A**: Auto-scroll form into view when keyboard appears |

**Date confirmed**: 2025-12-15
