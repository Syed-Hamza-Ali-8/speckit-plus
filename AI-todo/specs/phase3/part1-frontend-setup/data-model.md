# Data Model: Phase 3 Part 1 - Frontend Setup

**Feature**: Frontend TypeScript types and state schemas
**Date**: 2025-12-14

---

## 1. API Response Types

### 1.1 Authentication Types

```typescript
// src/types/auth.ts

/**
 * Login request payload
 */
export interface LoginRequest {
  email: string;
  password: string;
}

/**
 * Registration request payload
 */
export interface RegisterRequest {
  email: string;
  password: string;
}

/**
 * Authentication response from backend
 */
export interface AuthResponse {
  access_token: string;
  token_type: 'bearer';
}

/**
 * Decoded JWT payload (for client-side use)
 */
export interface JWTPayload {
  sub: string;      // User ID
  email: string;
  exp: number;      // Expiration timestamp
  iat: number;      // Issued at timestamp
}
```

### 1.2 Task Types

```typescript
// src/types/task.ts

/**
 * Task status enumeration
 */
export type TaskStatus = 'pending' | 'completed';

/**
 * Task entity from backend API
 * Note: user_id is NOT included (hidden by backend)
 */
export interface Task {
  id: string;
  title: string;
  description: string | null;
  status: TaskStatus;
  created_at: string;   // ISO 8601 datetime
  updated_at: string;   // ISO 8601 datetime
}

/**
 * Create task request payload
 */
export interface TaskCreate {
  title: string;
  description?: string;
}

/**
 * Update task request payload (partial)
 */
export interface TaskUpdate {
  title?: string;
  description?: string;
  status?: TaskStatus;
}

/**
 * Paginated response wrapper
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  limit: number;
  offset: number;
}

/**
 * Task list query parameters
 */
export interface TaskQueryParams {
  status?: TaskStatus;
  created_after?: string;   // ISO 8601 date
  created_before?: string;  // ISO 8601 date
  sort?: string;            // Format: "field:direction"
  limit?: number;           // Default: 20, Max: 100
  offset?: number;          // Default: 0
}
```

---

## 2. Client State Types

### 2.1 UI Store (Zustand)

```typescript
// src/stores/uiStore.ts

/**
 * Theme options
 */
export type Theme = 'light' | 'dark' | 'system';

/**
 * UI state managed by Zustand
 */
export interface UIState {
  // Theme
  theme: Theme;
  setTheme: (theme: Theme) => void;

  // Sidebar
  sidebarOpen: boolean;
  toggleSidebar: () => void;
  setSidebarOpen: (open: boolean) => void;

  // Modals
  createTaskModalOpen: boolean;
  setCreateTaskModalOpen: (open: boolean) => void;
  editTaskModalOpen: boolean;
  editingTaskId: string | null;
  openEditTaskModal: (taskId: string) => void;
  closeEditTaskModal: () => void;
}
```

### 2.2 Auth State (Derived)

```typescript
// src/hooks/useAuth.ts

/**
 * Auth state derived from localStorage and RTK Query
 */
export interface AuthState {
  isAuthenticated: boolean;
  token: string | null;
  isLoading: boolean;
  error: string | null;
}

/**
 * Auth hook return type
 */
export interface UseAuthReturn extends AuthState {
  login: (credentials: LoginRequest) => Promise<void>;
  register: (data: RegisterRequest) => Promise<void>;
  logout: () => void;
}
```

---

## 3. RTK Query Types

### 3.1 API Slice Types

```typescript
// src/services/api.ts

import { BaseQueryFn, FetchArgs, FetchBaseQueryError } from '@reduxjs/toolkit/query';

/**
 * Custom base query type with auth handling
 */
export type AuthBaseQuery = BaseQueryFn<
  string | FetchArgs,
  unknown,
  FetchBaseQueryError
>;

/**
 * API error response format
 */
export interface ApiError {
  status: number;
  data: {
    detail: string;
  };
}
```

### 3.2 Cache Tags

```typescript
/**
 * RTK Query cache tags for invalidation
 */
export const TAG_TYPES = {
  Task: 'Task',
  User: 'User',
} as const;

export type TagType = typeof TAG_TYPES[keyof typeof TAG_TYPES];
```

---

## 4. Route Types

### 4.1 Route Definitions

```typescript
// src/routes/index.tsx

/**
 * Application route paths
 */
export const ROUTES = {
  HOME: '/',
  LOGIN: '/login',
  REGISTER: '/register',
  TASKS: '/tasks',
} as const;

export type RoutePath = typeof ROUTES[keyof typeof ROUTES];
```

### 4.2 Protected Route Props

```typescript
// src/routes/ProtectedRoute.tsx

import { ReactNode } from 'react';

/**
 * Protected route component props
 */
export interface ProtectedRouteProps {
  children: ReactNode;
  redirectTo?: string;
}
```

---

## 5. Component Props Types

### 5.1 Layout Components

```typescript
// src/components/layout/Layout.tsx

import { ReactNode } from 'react';

export interface LayoutProps {
  children: ReactNode;
}

export interface HeaderProps {
  onMenuClick?: () => void;
}

export interface ThemeToggleProps {
  className?: string;
}
```

### 5.2 Form Components

```typescript
// src/components/auth/LoginForm.tsx (placeholder)

export interface LoginFormProps {
  onSuccess?: () => void;
  onError?: (error: string) => void;
}

export interface RegisterFormProps {
  onSuccess?: () => void;
  onError?: (error: string) => void;
}
```

---

## 6. State Relationships

```
┌─────────────────────────────────────────────────────────────┐
│                    Frontend State                            │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌───────────────────────────────────────────────────────┐  │
│  │                RTK Query Cache                         │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐   │  │
│  │  │ Auth State  │  │ Task List   │  │ Single Task │   │  │
│  │  │ (mutations) │  │ (query)     │  │ (query)     │   │  │
│  │  └─────────────┘  └─────────────┘  └─────────────┘   │  │
│  └───────────────────────────────────────────────────────┘  │
│                           │                                  │
│                           │ invalidates                      │
│                           ▼                                  │
│  ┌───────────────────────────────────────────────────────┐  │
│  │                Zustand Store                           │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐   │  │
│  │  │ Theme       │  │ Sidebar     │  │ Modals      │   │  │
│  │  │ (persisted) │  │ (runtime)   │  │ (runtime)   │   │  │
│  │  └─────────────┘  └─────────────┘  └─────────────┘   │  │
│  └───────────────────────────────────────────────────────┘  │
│                           │                                  │
│                           │ reads                            │
│                           ▼                                  │
│  ┌───────────────────────────────────────────────────────┐  │
│  │                localStorage                            │  │
│  │  ┌─────────────┐  ┌─────────────┐                    │  │
│  │  │ JWT Token   │  │ UI Theme    │                    │  │
│  │  │ (auth)      │  │ (ui-store)  │                    │  │
│  │  └─────────────┘  └─────────────┘                    │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

---

## 7. Validation Rules

### 7.1 Auth Validation

| Field | Rule | Error Message |
|-------|------|---------------|
| email | Valid email format | "Invalid email address" |
| email | Required | "Email is required" |
| password | Min 8 characters | "Password must be at least 8 characters" |
| password | Required | "Password is required" |

### 7.2 Task Validation

| Field | Rule | Error Message |
|-------|------|---------------|
| title | Required | "Title is required" |
| title | Max 200 chars | "Title must be 200 characters or less" |
| title | Min 1 char | "Title cannot be empty" |
| description | Max 1000 chars | "Description must be 1000 characters or less" |
| status | Enum value | "Invalid status" |
