# Specification: Phase 3 Part 1 - Frontend Setup

**Feature**: React 18 + Vite frontend scaffolding with auth and state management
**Version**: 1.0
**Date**: 2025-12-14
**Status**: Draft

---

## Section 1: Requirements

### 1.1 Functional Requirements

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-01 | React 18 SPA with Vite build tooling | Must |
| FR-02 | Tailwind CSS for styling with mobile-first responsive design | Must |
| FR-03 | shadcn/ui component library with Lucide React icons | Must |
| FR-04 | RTK Query for API state management (/auth/*, /tasks/*) | Must |
| FR-05 | Zustand for UI state (theme, modals, sidebar) | Must |
| FR-06 | JWT authentication with localStorage persistence | Must |
| FR-07 | Protected routes with automatic redirect for unauthenticated users | Must |
| FR-08 | Dark mode support via Tailwind `dark:` classes | Must |
| FR-09 | Vite dev server proxy to backend API | Must |
| FR-10 | 401 response handling with auto-logout | Must |

### 1.2 Non-Functional Requirements

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-01 | Dev server startup time | < 3 seconds |
| NFR-02 | Production bundle size | < 500KB gzipped |
| NFR-03 | Lighthouse performance score | > 90 |
| NFR-04 | TypeScript strict mode | Enabled |
| NFR-05 | Zero build errors | Required |

### 1.3 Tech Stack

| Layer | Technology | Version |
|-------|------------|---------|
| Framework | React | 18.3.x |
| Build Tool | Vite | 6.x |
| Language | TypeScript | 5.x |
| Styling | Tailwind CSS | 3.4.x |
| Components | shadcn/ui | Latest |
| Icons | Lucide React | 0.460.x |
| API State | RTK Query | 2.3.x |
| UI State | Zustand | 5.x |
| Routing | React Router | 6.28.x |

---

## Section 2: Architecture

### 2.1 Project Structure

```
phase3/frontend/
├── index.html
├── package.json
├── vite.config.ts
├── tailwind.config.ts
├── postcss.config.js
├── tsconfig.json
├── components.json           # shadcn config
├── .env.local
├── .env.example
└── src/
    ├── main.tsx              # App entry point
    ├── App.tsx               # Root component with providers
    ├── index.css             # Tailwind directives
    ├── vite-env.d.ts
    │
    ├── components/
    │   ├── ui/               # shadcn/ui components
    │   │   ├── button.tsx
    │   │   ├── card.tsx
    │   │   ├── input.tsx
    │   │   ├── label.tsx
    │   │   ├── dialog.tsx
    │   │   └── toast.tsx
    │   └── layout/
    │       ├── Layout.tsx    # App shell wrapper
    │       ├── Header.tsx    # Navigation + theme toggle
    │       └── ThemeToggle.tsx
    │
    ├── hooks/
    │   ├── useAuth.ts        # Login/register/logout helpers
    │   └── useTheme.ts       # Theme switching hook
    │
    ├── lib/
    │   ├── store.ts          # Redux store configuration
    │   └── utils.ts          # cn() helper for Tailwind classes
    │
    ├── pages/
    │   ├── LoginPage.tsx     # Login form (placeholder)
    │   ├── RegisterPage.tsx  # Register form (placeholder)
    │   └── TasksPage.tsx     # Task list (placeholder)
    │
    ├── routes/
    │   ├── index.tsx         # Route definitions
    │   └── ProtectedRoute.tsx # Auth guard component
    │
    ├── services/
    │   ├── api.ts            # RTK Query base configuration
    │   ├── authApi.ts        # Auth endpoint definitions
    │   └── taskApi.ts        # Task endpoint definitions
    │
    ├── stores/
    │   └── uiStore.ts        # Zustand UI state store
    │
    └── types/
        ├── auth.ts           # Auth-related types
        └── task.ts           # Task-related types
```

### 2.2 Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│                        Frontend                              │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────┐    ┌───────────────┐    ┌──────────────────┐  │
│  │  Pages   │───▶│  RTK Query    │───▶│  Backend API     │  │
│  │          │    │  (API State)  │    │  localhost:8000  │  │
│  └──────────┘    └───────────────┘    └──────────────────┘  │
│       │                                        │             │
│       │          ┌───────────────┐             │             │
│       └─────────▶│   Zustand     │             │             │
│                  │  (UI State)   │             │             │
│                  └───────────────┘             │             │
│                                                │             │
│  ┌──────────────────────────────────────────┐  │             │
│  │             localStorage                  │  │             │
│  │  - JWT token                             │◀─┘             │
│  │  - Theme preference                      │                │
│  └──────────────────────────────────────────┘                │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### 2.3 Authentication Flow

```
┌────────┐     ┌───────────┐     ┌─────────────┐     ┌─────────┐
│ Login  │────▶│ RTK Query │────▶│ POST /auth/ │────▶│ Backend │
│ Form   │     │ Mutation  │     │ login       │     │         │
└────────┘     └───────────┘     └─────────────┘     └─────────┘
                    │                                      │
                    │ ◀────────── access_token ───────────┘
                    │
                    ▼
            ┌───────────────┐
            │ localStorage  │
            │ set('token')  │
            └───────────────┘
                    │
                    ▼
            ┌───────────────┐
            │ Navigate to   │
            │ /tasks        │
            └───────────────┘
```

### 2.4 Protected Route Flow

```
┌─────────────────┐
│ Navigate to     │
│ /tasks          │
└────────┬────────┘
         │
         ▼
┌─────────────────┐     No      ┌─────────────────┐
│ Token in        │────────────▶│ Redirect to     │
│ localStorage?   │             │ /login          │
└────────┬────────┘             └─────────────────┘
         │ Yes
         ▼
┌─────────────────┐
│ Render TasksPage│
└─────────────────┘
```

---

## Section 3: Implementation Details

### 3.1 RTK Query Configuration

```typescript
// src/services/api.ts
import { createApi, fetchBaseQuery } from '@reduxjs/toolkit/query/react';

const baseQuery = fetchBaseQuery({
  baseUrl: import.meta.env.VITE_API_URL || '/api',
  prepareHeaders: (headers) => {
    const token = localStorage.getItem('token');
    if (token) {
      headers.set('Authorization', `Bearer ${token}`);
    }
    return headers;
  },
});

const baseQueryWithReauth = async (args, api, extraOptions) => {
  const result = await baseQuery(args, api, extraOptions);
  if (result.error?.status === 401) {
    localStorage.removeItem('token');
    window.location.href = '/login';
  }
  return result;
};

export const api = createApi({
  reducerPath: 'api',
  baseQuery: baseQueryWithReauth,
  tagTypes: ['Task'],
  endpoints: () => ({}),
});
```

### 3.2 Auth API Endpoints

```typescript
// src/services/authApi.ts
import { api } from './api';
import { LoginRequest, RegisterRequest, AuthResponse } from '../types/auth';

export const authApi = api.injectEndpoints({
  endpoints: (build) => ({
    login: build.mutation<AuthResponse, LoginRequest>({
      query: (credentials) => ({
        url: '/auth/login',
        method: 'POST',
        body: credentials,
      }),
    }),
    register: build.mutation<AuthResponse, RegisterRequest>({
      query: (data) => ({
        url: '/auth/register',
        method: 'POST',
        body: data,
      }),
    }),
  }),
});

export const { useLoginMutation, useRegisterMutation } = authApi;
```

### 3.3 Task API Endpoints

```typescript
// src/services/taskApi.ts
import { api } from './api';
import { Task, TaskCreate, TaskUpdate, PaginatedResponse } from '../types/task';

export const taskApi = api.injectEndpoints({
  endpoints: (build) => ({
    getTasks: build.query<PaginatedResponse<Task>, TaskQueryParams>({
      query: (params) => ({
        url: '/tasks',
        params,
      }),
      providesTags: ['Task'],
    }),
    createTask: build.mutation<Task, TaskCreate>({
      query: (task) => ({
        url: '/tasks',
        method: 'POST',
        body: task,
      }),
      invalidatesTags: ['Task'],
    }),
    updateTask: build.mutation<Task, { id: string } & TaskUpdate>({
      query: ({ id, ...patch }) => ({
        url: `/tasks/${id}`,
        method: 'PATCH',
        body: patch,
      }),
      invalidatesTags: ['Task'],
    }),
    deleteTask: build.mutation<void, string>({
      query: (id) => ({
        url: `/tasks/${id}`,
        method: 'DELETE',
      }),
      invalidatesTags: ['Task'],
    }),
  }),
});

export const {
  useGetTasksQuery,
  useCreateTaskMutation,
  useUpdateTaskMutation,
  useDeleteTaskMutation,
} = taskApi;
```

### 3.4 Zustand UI Store

```typescript
// src/stores/uiStore.ts
import { create } from 'zustand';
import { persist } from 'zustand/middleware';

interface UIState {
  theme: 'light' | 'dark' | 'system';
  sidebarOpen: boolean;
  setTheme: (theme: UIState['theme']) => void;
  toggleSidebar: () => void;
}

export const useUIStore = create<UIState>()(
  persist(
    (set) => ({
      theme: 'system',
      sidebarOpen: true,
      setTheme: (theme) => set({ theme }),
      toggleSidebar: () => set((state) => ({ sidebarOpen: !state.sidebarOpen })),
    }),
    {
      name: 'ui-storage',
      partialize: (state) => ({ theme: state.theme }),
    }
  )
);
```

### 3.5 Vite Configuration

```typescript
// vite.config.ts
import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import path from 'path';

export default defineConfig({
  plugins: [react()],
  resolve: {
    alias: {
      '@': path.resolve(__dirname, './src'),
    },
  },
  server: {
    port: 5173,
    proxy: {
      '/api': {
        target: 'http://localhost:8000',
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/api/, ''),
      },
    },
  },
});
```

### 3.6 Tailwind Configuration

```typescript
// tailwind.config.ts
import type { Config } from 'tailwindcss';

const config: Config = {
  darkMode: 'class',
  content: [
    './index.html',
    './src/**/*.{js,ts,jsx,tsx}',
  ],
  theme: {
    extend: {
      // Custom theme extensions
    },
  },
  plugins: [],
};

export default config;
```

### 3.7 Environment Variables

```bash
# .env.local (development)
VITE_API_URL=http://localhost:8000

# .env.production
VITE_API_URL=https://api.yourdomain.com
```

### 3.8 TypeScript Types

```typescript
// src/types/auth.ts
export interface LoginRequest {
  email: string;
  password: string;
}

export interface RegisterRequest {
  email: string;
  password: string;
}

export interface AuthResponse {
  access_token: string;
  token_type: string;
}

// src/types/task.ts
export type TaskStatus = 'pending' | 'completed';

export interface Task {
  id: string;
  title: string;
  description: string | null;
  status: TaskStatus;
  created_at: string;
  updated_at: string;
}

export interface TaskCreate {
  title: string;
  description?: string;
}

export interface TaskUpdate {
  title?: string;
  description?: string;
  status?: TaskStatus;
}

export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  limit: number;
  offset: number;
}

export interface TaskQueryParams {
  status?: TaskStatus;
  created_after?: string;
  created_before?: string;
  sort?: string;
  limit?: number;
  offset?: number;
}
```

---

## Section 4: Success Criteria

### 4.1 Acceptance Tests

| ID | Test | Expected Result | Priority |
|----|------|-----------------|----------|
| AT-01 | Run `npm install && npm run dev` | Dev server starts at localhost:5173 | Must |
| AT-02 | Navigate to localhost:5173 | React app renders without errors | Must |
| AT-03 | `curl localhost:5173/api/auth/login -X POST` | Proxies to backend, returns response | Must |
| AT-04 | Execute login mutation via RTK Query | JWT saved to localStorage | Must |
| AT-05 | Toggle dark mode | Theme switches, persists on reload | Must |
| AT-06 | Access /tasks without token | Redirects to /login | Must |
| AT-07 | Access /tasks with valid token | TasksPage renders | Must |
| AT-08 | Receive 401 from API | Token cleared, redirects to /login | Must |
| AT-09 | Resize to mobile viewport | Layout remains responsive | Must |
| AT-10 | Run `npm run build` | Production bundle builds (0 errors) | Must |

### 4.2 Verification Commands

```bash
# AT-01: Dev server startup
cd phase3/frontend && npm install && npm run dev
# Expected: Server running at http://localhost:5173

# AT-03: Proxy verification
curl -X POST http://localhost:5173/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"password123"}'
# Expected: Backend auth response (or 401)

# AT-10: Production build
npm run build
# Expected: dist/ folder created, 0 TypeScript errors
```

### 4.3 Quality Gates

| Gate | Requirement |
|------|-------------|
| TypeScript | `npm run build` passes with strict mode |
| Lint | `npm run lint` passes (if configured) |
| Bundle Size | `dist/` < 500KB gzipped |
| Dev Experience | Hot reload works within 500ms |

### 4.4 Definition of Done

- [ ] All 10 acceptance tests pass
- [ ] TypeScript strict mode enabled and compiling
- [ ] Vite proxy correctly forwards /api/* to backend
- [ ] RTK Query DevTools shows API slice in Redux DevTools
- [ ] Dark mode toggle functional with localStorage persistence
- [ ] ProtectedRoute redirects unauthenticated users
- [ ] Mobile responsive on viewports 320px - 1920px
- [ ] Production build completes without errors

---

## Clarifications

### Session 2025-12-14

- Q: Dark Mode Default: 'system' preference OR 'light' initially? → A: 'system' (respect OS dark/light preference)
- Q: ProtectedRoute Behavior: Check localStorage JWT OR call /auth/me first? → A: localStorage JWT check only (fast, no network call)

---

## Appendix A: Dependencies

```json
{
  "dependencies": {
    "react": "^18.3.1",
    "react-dom": "^18.3.1",
    "react-router-dom": "^6.28.0",
    "@reduxjs/toolkit": "^2.3.0",
    "react-redux": "^9.1.2",
    "zustand": "^5.0.0",
    "lucide-react": "^0.460.0",
    "clsx": "^2.1.1",
    "tailwind-merge": "^2.5.0",
    "class-variance-authority": "^0.7.1"
  },
  "devDependencies": {
    "@types/react": "^18.3.12",
    "@types/react-dom": "^18.3.1",
    "@vitejs/plugin-react": "^4.3.3",
    "autoprefixer": "^10.4.20",
    "postcss": "^8.4.49",
    "tailwindcss": "^3.4.15",
    "typescript": "^5.6.3",
    "vite": "^6.0.1"
  }
}
```

---

## Appendix B: Related Documents

- [Implementation Plan](./plan.md)
- [Backend API Spec](../../phase2/part3-tasks-crud/spec.md)
- [OpenAPI Contract](../../phase2/part3-tasks-crud/contracts/tasks-api.yaml)
