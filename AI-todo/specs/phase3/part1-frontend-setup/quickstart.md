# Quickstart: Phase 3 Part 1 - Frontend Setup

**Feature**: React 18 + Vite frontend scaffolding
**Prerequisites**: Node.js 20+, npm 10+, Backend running on localhost:8000

---

## 1. Quick Setup (5 minutes)

```bash
# Navigate to frontend directory
cd phase3/frontend

# Install dependencies
npm install

# Start development server
npm run dev
```

**Expected**: Browser opens to http://localhost:5173 with React app

---

## 2. Project Scaffolding Commands

### 2.1 Create Vite Project

```bash
# From project root
cd phase3
npm create vite@latest frontend -- --template react-ts
cd frontend
```

### 2.2 Install Core Dependencies

```bash
# Styling
npm install -D tailwindcss postcss autoprefixer
npx tailwindcss init -p

# State Management
npm install @reduxjs/toolkit react-redux zustand

# Routing
npm install react-router-dom

# Icons & Utils
npm install lucide-react clsx tailwind-merge class-variance-authority
```

### 2.3 Initialize shadcn/ui

```bash
npx shadcn-ui@latest init

# When prompted:
# - Style: Default
# - Base color: Slate
# - CSS variables: Yes
# - tailwind.config.ts location: tailwind.config.ts
# - components.json location: components.json
# - Components location: src/components/ui
# - Utils location: src/lib/utils

# Install base components
npx shadcn-ui@latest add button card input label dialog toast
```

---

## 3. Configuration Files

### 3.1 vite.config.ts

```typescript
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

### 3.2 tailwind.config.ts

```typescript
import type { Config } from 'tailwindcss';

const config: Config = {
  darkMode: 'class',
  content: [
    './index.html',
    './src/**/*.{js,ts,jsx,tsx}',
  ],
  theme: {
    extend: {},
  },
  plugins: [],
};

export default config;
```

### 3.3 tsconfig.json (paths section)

```json
{
  "compilerOptions": {
    "baseUrl": ".",
    "paths": {
      "@/*": ["./src/*"]
    }
  }
}
```

### 3.4 .env.local

```bash
VITE_API_URL=http://localhost:8000
```

---

## 4. Folder Structure Creation

```bash
# Create directory structure
mkdir -p src/{components/{ui,layout},hooks,lib,pages,routes,services,stores,types}

# Create placeholder files
touch src/components/layout/{Layout,Header,ThemeToggle}.tsx
touch src/hooks/{useAuth,useTheme}.ts
touch src/lib/{store,utils}.ts
touch src/pages/{LoginPage,RegisterPage,TasksPage}.tsx
touch src/routes/{index,ProtectedRoute}.tsx
touch src/services/{api,authApi,taskApi}.ts
touch src/stores/uiStore.ts
touch src/types/{auth,task}.ts
```

---

## 5. Verification Steps

### 5.1 Dev Server Check

```bash
npm run dev
# Expected: Server starts at http://localhost:5173
```

### 5.2 Proxy Check

```bash
# In another terminal, with backend running
curl -X POST http://localhost:5173/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"wrong"}'
# Expected: {"detail":"Invalid credentials"} or similar backend response
```

### 5.3 Build Check

```bash
npm run build
# Expected: dist/ folder created, 0 TypeScript errors
```

### 5.4 TypeScript Check

```bash
npx tsc --noEmit
# Expected: No errors
```

---

## 6. Development Commands

| Command | Description |
|---------|-------------|
| `npm run dev` | Start dev server with HMR |
| `npm run build` | Create production build |
| `npm run preview` | Preview production build |
| `npm run lint` | Run ESLint |
| `npx tsc --noEmit` | Type check without build |

---

## 7. Common Issues & Solutions

### Issue: CORS errors in browser

**Solution**: Ensure backend CORS is configured for localhost:5173
```python
# In FastAPI main.py
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### Issue: Vite proxy not working

**Solution**: Check target URL matches backend port
```typescript
proxy: {
  '/api': {
    target: 'http://localhost:8000',  // Verify this port
    changeOrigin: true,
  },
}
```

### Issue: shadcn components not styled

**Solution**: Ensure Tailwind directives in index.css
```css
@tailwind base;
@tailwind components;
@tailwind utilities;
```

### Issue: Path alias not resolving

**Solution**: Add to both tsconfig.json AND vite.config.ts
```typescript
// vite.config.ts
resolve: {
  alias: {
    '@': path.resolve(__dirname, './src'),
  },
},
```

---

## 8. Next Steps After Setup

1. **Phase 3 Part 2**: Implement auth forms (Login, Register)
2. **Phase 3 Part 3**: Implement task CRUD UI
3. **Phase 3 Part 4**: Add error handling, loading states, toasts

---

## 9. Useful Resources

- [Vite Docs](https://vitejs.dev/)
- [React Router v6](https://reactrouter.com/)
- [RTK Query](https://redux-toolkit.js.org/rtk-query/overview)
- [Zustand](https://docs.pmnd.rs/zustand)
- [shadcn/ui](https://ui.shadcn.com/)
- [Tailwind CSS](https://tailwindcss.com/)
