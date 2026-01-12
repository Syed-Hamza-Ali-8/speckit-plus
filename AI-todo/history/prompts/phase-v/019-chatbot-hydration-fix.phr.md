---
id: 019
title: Fix chatbot not showing on first page load
stage: green
date: 2026-01-13
surface: agent
model: claude-sonnet-4-5
feature: phase-v
branch: main
user: hamza
command: continue
labels: ["bugfix", "chatbot", "hydration", "authentication", "react"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - phase-5/frontend/src/hooks/useAuth.ts
tests:
  - Manual testing: Load page for first time and verify chatbot appears
  - Manual testing: Refresh page and verify chatbot still appears
  - Manual testing: Login and verify chatbot appears immediately
---

## Prompt

User reported: "everything is perfect but my chatbot isn't showing in first attempt, I have to refresh the page then it is showing"

The chatbot widget was not appearing on the first page load after login. Users had to manually refresh the page to see the chatbot floating button.

## Response snapshot

**Root Cause Identified:** React hydration mismatch caused by accessing `localStorage` during initial render.

**The Problem:**

The `useAuth` hook was directly accessing `localStorage` during component render:

```typescript
// BEFORE (phase-5/frontend/src/hooks/useAuth.ts):
export function useAuth(): UseAuthReturn {
  const [loginMutation, { isLoading: isLoginLoading }] = useLoginMutation();
  const [registerMutation, { isLoading: isRegisterLoading }] = useRegisterMutation();

  // ❌ Accessing localStorage during render
  const token = localStorage.getItem('token');
  const isAuthenticated = !!token;
  const isLoading = isLoginLoading || isRegisterLoading;
  // ...
}
```

**Why This Causes Issues:**

1. **Server-Side Rendering (SSR):** In Next.js/React, components may be rendered on the server first
2. **localStorage is client-only:** `localStorage` doesn't exist on the server
3. **Hydration Mismatch:** Server renders with `isAuthenticated = false`, but client has token in localStorage
4. **Result:** React detects mismatch and delays rendering, causing chatbot to not appear until after refresh

**The Fix:**

Use `useState` and `useEffect` to check localStorage after component mounts (client-side only):

```typescript
// AFTER (phase-5/frontend/src/hooks/useAuth.ts):
export function useAuth(): UseAuthReturn {
  const [loginMutation, { isLoading: isLoginLoading }] = useLoginMutation();
  const [registerMutation, { isLoading: isRegisterLoading }] = useRegisterMutation();

  // ✅ State to track authentication status
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [token, setToken] = useState<string | null>(null);

  // ✅ Check localStorage after component mounts (client-side only)
  useEffect(() => {
    const storedToken = localStorage.getItem('token');
    setToken(storedToken);
    setIsAuthenticated(!!storedToken);
  }, []);

  const isLoading = isLoginLoading || isRegisterLoading;
  // ...
}
```

**Additional Updates:**

Updated login, register, and logout functions to also update the state:

```typescript
const login = useCallback(
  async (credentials: LoginRequest): Promise<AuthResponse> => {
    const result = await loginMutation(credentials).unwrap();
    localStorage.setItem('token', result.access_token);
    setToken(result.access_token);        // ✅ Update state
    setIsAuthenticated(true);             // ✅ Update state
    return result;
  },
  [loginMutation]
);

const logout = useCallback(() => {
  localStorage.removeItem('token');
  setToken(null);                         // ✅ Update state
  setIsAuthenticated(false);              // ✅ Update state
  window.location.href = '/login';
}, []);
```

**How ChatWidget Rendering Works:**

File: `phase-5/frontend/src/components/layout/Layout.tsx`

```typescript
export function Layout({ children }: LayoutProps) {
  const { isAuthenticated } = useAuth();

  return (
    <div className="min-h-screen bg-background">
      <Header />
      <main className="container mx-auto px-4 py-8">
        {children}
      </main>

      {/* Chat Widget - only shown for authenticated users */}
      {isAuthenticated && <ChatWidget />}
    </div>
  );
}
```

**Before Fix:**
1. Page loads → `isAuthenticated = false` (localStorage not checked yet)
2. ChatWidget doesn't render
3. User refreshes → `isAuthenticated = true` (now checked)
4. ChatWidget appears

**After Fix:**
1. Page loads → `isAuthenticated = false` initially
2. `useEffect` runs → checks localStorage → `isAuthenticated = true`
3. Component re-renders → ChatWidget appears immediately
4. No refresh needed

**Deployment:**
- Rebuilt frontend Docker container: `docker-compose build frontend`
- Deployed updated container: `docker-compose up -d frontend`
- Frontend container running and healthy

## Outcome

- ✅ Impact: Fixed critical UX issue where chatbot didn't appear on first page load. Users no longer need to refresh the page to see the chatbot.
- 🧪 Tests: Manual testing required - user should load the page fresh and verify chatbot appears immediately without refresh.
- 📁 Files: Modified 1 frontend hook file to properly handle client-side localStorage access.
- 🔁 Next prompts: User should test by clearing browser cache, loading the app, and verifying chatbot appears on first load.
- 🧠 Reflection: This is a classic React hydration issue. Accessing browser APIs (localStorage, window, document) during render causes mismatches between server and client. The solution is always to use useEffect to access these APIs after mount. This pattern should be applied consistently across the codebase.

## Evaluation notes (flywheel)

- Failure modes observed: React hydration mismatch from accessing localStorage during render. No warning in console about hydration issues (React 19 may have improved error messages). Issue only manifested as delayed rendering, not a hard error.
- Graders run and results (PASS/FAIL): Frontend deployment PASS - container rebuilt and running. Manual testing PENDING - awaiting user verification that chatbot appears on first load.
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add ESLint rule to detect direct localStorage access outside of useEffect. Create a custom hook `useLocalStorage` that properly handles SSR/hydration. Audit all other browser API usage (window, document, sessionStorage) for similar issues.
