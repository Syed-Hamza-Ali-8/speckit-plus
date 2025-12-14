# Implementation Plan: Phase 3 Part 1 - Frontend Setup

**Feature**: Frontend scaffolding with React 18 + Vite + Tailwind + RTK Query + Zustand
**Prerequisites**: Backend API running at localhost:8000 (Phase 2 complete)
**Output**: Working frontend dev environment with auth flow and dark mode

---

## 1. Implementation Phases

### Phase 1: Vite + React + TypeScript Setup

**Purpose**: Bootstrap React 18 project with TypeScript and Tailwind CSS

**Deliverables**:
- `phase3/frontend/` directory with Vite React-TS template
- Tailwind CSS configured with `tailwind.config.ts`
- PostCSS setup for Tailwind processing
- Vite dev server proxy to backend API (localhost:8000)
- Base TypeScript config (`tsconfig.json`)

**Key Files**:
```
phase3/frontend/
├── index.html
├── package.json
├── vite.config.ts          # Proxy config
├── tailwind.config.ts      # Dark mode: 'class'
├── postcss.config.js
├── tsconfig.json
├── .env.local              # VITE_API_URL
└── src/
    ├── main.tsx
    ├── App.tsx
    └── index.css           # Tailwind directives
```

**Checkpoint**: `npm run dev` → localhost:5173 shows React app

---

### Phase 2: shadcn/ui + Lucide Icons

**Purpose**: Install component library and icon set

**Deliverables**:
- shadcn/ui initialized with `components.json`
- Base components installed: Button, Card, Input, Label, Dialog, Toast
- Lucide React icons available
- `lib/utils.ts` with `cn()` helper

**Key Files**:
```
src/
├── components/
│   └── ui/
│       ├── button.tsx
│       ├── card.tsx
│       ├── input.tsx
│       ├── label.tsx
│       ├── dialog.tsx
│       └── toast.tsx
└── lib/
    └── utils.ts            # cn() helper
```

**Checkpoint**: Can import and render `<Button>` component

---

### Phase 3: RTK Query API Store

**Purpose**: Configure Redux Toolkit Query for API communication

**Deliverables**:
- Redux store with RTK Query middleware
- Base API with auth header injection
- Auth endpoints: `login`, `register`
- Task endpoints: `getTasks`, `createTask`, `updateTask`, `deleteTask`
- TypeScript types for API responses

**Key Files**:
```
src/
├── lib/
│   └── store.ts            # Redux store config
├── services/
│   ├── api.ts              # Base RTK Query API
│   ├── authApi.ts          # Auth endpoints
│   └── taskApi.ts          # Task endpoints
└── types/
    ├── auth.ts             # LoginRequest, AuthResponse
    └── task.ts             # Task, TaskCreate, TaskUpdate, PaginatedResponse
```

**API Configuration**:
```typescript
// Base query with auth header
const baseQuery = fetchBaseQuery({
  baseUrl: import.meta.env.VITE_API_URL || '/api',
  prepareHeaders: (headers) => {
    const token = localStorage.getItem('token');
    if (token) headers.set('Authorization', `Bearer ${token}`);
    return headers;
  },
});

// 401 handling
const baseQueryWithReauth = async (args, api, extraOptions) => {
  const result = await baseQuery(args, api, extraOptions);
  if (result.error?.status === 401) {
    localStorage.removeItem('token');
    window.location.href = '/login';
  }
  return result;
};
```

**Checkpoint**: RTK DevTools shows API slice, mutations callable

---

### Phase 4: Zustand UI Store + Auth Hooks

**Purpose**: Lightweight UI state management and auth utilities

**Deliverables**:
- Zustand store for UI state (theme, sidebar, modals)
- `useAuth()` hook for auth operations
- Token persistence helpers
- Auth state derivation from localStorage

**Key Files**:
```
src/
├── stores/
│   └── uiStore.ts          # Theme, sidebar, modal state
└── hooks/
    ├── useAuth.ts          # Login, logout, isAuthenticated
    └── useTheme.ts         # Theme toggle helper
```

**Zustand Store**:
```typescript
interface UIState {
  theme: 'light' | 'dark' | 'system';
  sidebarOpen: boolean;
  setTheme: (theme: UIState['theme']) => void;
  toggleSidebar: () => void;
}
```

**Auth Hook**:
```typescript
const useAuth = () => {
  const [login] = useLoginMutation();
  const [register] = useRegisterMutation();

  const isAuthenticated = !!localStorage.getItem('token');

  const handleLogin = async (credentials) => {
    const result = await login(credentials).unwrap();
    localStorage.setItem('token', result.access_token);
    return result;
  };

  const logout = () => {
    localStorage.removeItem('token');
    window.location.href = '/login';
  };

  return { login: handleLogin, register, logout, isAuthenticated };
};
```

**Checkpoint**: `useAuth().isAuthenticated` returns correct value

---

### Phase 5: Routing + ProtectedRoute

**Purpose**: Client-side routing with auth guards

**Deliverables**:
- React Router v6 setup
- Route definitions for `/login`, `/register`, `/tasks`
- `ProtectedRoute` component for auth guards
- Redirect logic for authenticated/unauthenticated users

**Key Files**:
```
src/
├── routes/
│   ├── index.tsx           # Route definitions
│   └── ProtectedRoute.tsx  # Auth guard wrapper
└── pages/
    ├── LoginPage.tsx       # Placeholder
    ├── RegisterPage.tsx    # Placeholder
    └── TasksPage.tsx       # Placeholder
```

**Route Structure**:
```typescript
<Routes>
  <Route path="/login" element={<LoginPage />} />
  <Route path="/register" element={<RegisterPage />} />
  <Route path="/" element={<ProtectedRoute><TasksPage /></ProtectedRoute>} />
  <Route path="/tasks" element={<ProtectedRoute><TasksPage /></ProtectedRoute>} />
  <Route path="*" element={<Navigate to="/" />} />
</Routes>
```

**ProtectedRoute**:
```typescript
const ProtectedRoute = ({ children }) => {
  const { isAuthenticated } = useAuth();

  if (!isAuthenticated) {
    return <Navigate to="/login" replace />;
  }

  return children;
};
```

**Checkpoint**: Unauthenticated user redirected from `/tasks` to `/login`

---

### Phase 6: Basic Layout + Dark Mode

**Purpose**: Application shell with theme switching

**Deliverables**:
- `Layout` component with header and main content area
- Dark mode toggle using Zustand + Tailwind
- Theme persistence in localStorage
- System theme detection

**Key Files**:
```
src/
├── components/
│   └── layout/
│       ├── Layout.tsx      # App shell
│       ├── Header.tsx      # Nav + theme toggle
│       └── ThemeToggle.tsx # Dark/light switch
└── App.tsx                 # Layout wrapper
```

**Theme Implementation**:
```typescript
// Apply theme class to document
useEffect(() => {
  const root = document.documentElement;
  if (theme === 'dark') {
    root.classList.add('dark');
  } else if (theme === 'light') {
    root.classList.remove('dark');
  } else {
    // System preference
    const isDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
    root.classList.toggle('dark', isDark);
  }
}, [theme]);
```

**Checkpoint**: Theme toggle switches between light/dark, persists on reload

---

## 2. Dependency Graph

```
Phase 1: Vite + React + Tailwind
         │
         ▼
Phase 2: shadcn/ui + Lucide
         │
         ▼
Phase 3: RTK Query API ─────────┐
         │                      │
         ▼                      │
Phase 4: Zustand + Auth Hooks ◄─┘
         │
         ▼
Phase 5: Routing + ProtectedRoute
         │
         ▼
Phase 6: Layout + Dark Mode
```

**Critical Path**: Phases must execute sequentially (each depends on previous)

---

## 3. Complete File Structure

```
phase3/frontend/
├── index.html
├── package.json
├── vite.config.ts
├── tailwind.config.ts
├── postcss.config.js
├── tsconfig.json
├── tsconfig.node.json
├── components.json              # shadcn config
├── .env.local
├── .env.example
├── .gitignore
└── src/
    ├── main.tsx
    ├── App.tsx
    ├── index.css
    ├── vite-env.d.ts
    ├── components/
    │   ├── ui/                  # shadcn components
    │   │   ├── button.tsx
    │   │   ├── card.tsx
    │   │   ├── input.tsx
    │   │   ├── label.tsx
    │   │   ├── dialog.tsx
    │   │   └── toast.tsx
    │   └── layout/
    │       ├── Layout.tsx
    │       ├── Header.tsx
    │       └── ThemeToggle.tsx
    ├── hooks/
    │   ├── useAuth.ts
    │   └── useTheme.ts
    ├── lib/
    │   ├── store.ts             # Redux store
    │   └── utils.ts             # cn() helper
    ├── pages/
    │   ├── LoginPage.tsx
    │   ├── RegisterPage.tsx
    │   └── TasksPage.tsx
    ├── routes/
    │   ├── index.tsx
    │   └── ProtectedRoute.tsx
    ├── services/
    │   ├── api.ts
    │   ├── authApi.ts
    │   └── taskApi.ts
    ├── stores/
    │   └── uiStore.ts
    └── types/
        ├── auth.ts
        └── task.ts
```

---

## 4. Success Criteria

| Criterion | Verification |
|-----------|--------------|
| Dev server runs | `npm run dev` → localhost:5173 |
| Vite proxy works | `curl localhost:5173/api/health` → backend response |
| shadcn components | `<Button>` renders correctly |
| RTK Query setup | Redux DevTools shows API slice |
| Auth flow | Login stores token, logout clears |
| Protected routes | `/tasks` redirects to `/login` when unauthenticated |
| Dark mode | Toggle switches theme, persists on reload |
| TypeScript | `npm run build` completes without errors |

---

## 5. Dependencies (package.json)

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

## 6. Environment Configuration

```bash
# .env.local (development)
VITE_API_URL=http://localhost:8000

# .env.production
VITE_API_URL=https://api.yourdomain.com
```

**Vite Proxy** (development only):
```typescript
// vite.config.ts
export default defineConfig({
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

---

## 7. Risk Mitigation

| Risk | Mitigation |
|------|------------|
| CORS issues | Vite proxy handles dev; backend CORS config for production |
| Token expiry | 401 handler clears token and redirects |
| Theme flash | Load theme from localStorage before render |
| Bundle size | Tree-shaking via ES modules, Lucide icons are tree-shakeable |

---

## 8. Constitution Compliance

| Rule | Status | Notes |
|------|--------|-------|
| Spec-Driven Development | COMPLIANT | spec.md exists |
| Phase II Technology Stack | COMPLIANT | React/Vite allowed per constitution |
| TypeScript Strict Mode | COMPLIANT | Enabled in tsconfig.json |
| Clean Architecture | COMPLIANT | Clear separation: services, hooks, stores, types |
| Testing Requirements | PENDING | Tests planned for Part 4 |
| Interface Contracts | COMPLIANT | TypeScript interfaces defined |

---

## 9. Related Documents

- [spec.md](./spec.md) - Formal specification
- [research.md](./research.md) - Technology research and decisions
- [data-model.md](./data-model.md) - TypeScript type definitions
- [quickstart.md](./quickstart.md) - Setup guide

---

## 10. Next Steps After This Phase

1. **Phase 3 Part 2**: Auth UI (Login/Register forms with validation)
2. **Phase 3 Part 3**: Task CRUD UI (TaskList, TaskCard, TaskForm)
3. **Phase 3 Part 4**: Polish (Loading states, error handling, toasts)
