# Research: Phase 3 Part 1 - Frontend Setup

**Feature**: React 18 + Vite frontend scaffolding
**Date**: 2025-12-14
**Status**: Complete

---

## Research Questions & Findings

### RQ-01: Vite vs Next.js for this use case

**Decision**: Vite + React 18

**Rationale**:
- Backend API already exists (FastAPI) - no need for Next.js API routes
- Pure SPA pattern sufficient for task management app
- Faster dev server startup (< 1s vs 3-5s)
- Simpler mental model without SSR complexity
- Smaller production bundle

**Alternatives Considered**:
- Next.js 15: Overkill for SPA, adds unnecessary SSR complexity
- Create React App: Deprecated, slower than Vite

---

### RQ-02: State Management Architecture

**Decision**: RTK Query (API) + Zustand (UI)

**Rationale**:
- RTK Query provides automatic caching, request deduplication, optimistic updates
- Zustand is lightweight (~1KB) for simple UI state
- Clear separation: server state vs client state
- Both have excellent TypeScript support

**Alternatives Considered**:
- Redux only: Too much boilerplate for UI state
- React Query + Context: Similar to chosen, but RTK integrates better with Redux DevTools
- Zustand only: Lacks built-in API caching features

---

### RQ-03: JWT Storage Strategy

**Decision**: localStorage with 401 auto-logout

**Rationale**:
- Backend doesn't support httpOnly cookie refresh flow
- localStorage provides simplest implementation
- 401 interceptor handles token expiry gracefully
- XSS mitigation via Content Security Policy

**Alternatives Considered**:
- httpOnly cookies: Backend doesn't support this flow
- sessionStorage: Lost on tab close, poor UX
- In-memory only: Lost on refresh, poor UX

**Security Mitigations**:
1. Never store sensitive data beyond token
2. Implement proper CSP headers
3. Clear token on 401 response
4. Consider token encryption in future phases

---

### RQ-04: Component Library Selection

**Decision**: shadcn/ui + Lucide React

**Rationale**:
- Copy-paste ownership model (no dependency lock-in)
- Built on Radix UI primitives (accessibility)
- Tailwind CSS integration
- Highly customizable
- Tree-shakeable icons with Lucide

**Alternatives Considered**:
- Material UI: Heavy bundle, opinionated styling
- Chakra UI: Good but larger than shadcn
- Headless UI: Less components out of box

---

### RQ-05: Dark Mode Implementation

**Decision**: Tailwind `dark:` class + Zustand persist

**Rationale**:
- Native Tailwind support with `darkMode: 'class'`
- Zustand persist middleware for localStorage
- System preference detection via `prefers-color-scheme`
- No flash of unstyled content with proper initialization

**Implementation Pattern**:
```typescript
// Theme initialization (before React renders)
const theme = localStorage.getItem('theme') || 'system';
if (theme === 'dark' || (theme === 'system' && window.matchMedia('(prefers-color-scheme: dark)').matches)) {
  document.documentElement.classList.add('dark');
}
```

---

### RQ-06: Vite Proxy Configuration

**Decision**: Dev server proxy to localhost:8000

**Rationale**:
- Avoids CORS issues in development
- Mimics production environment
- Simple configuration

**Configuration**:
```typescript
server: {
  proxy: {
    '/api': {
      target: 'http://localhost:8000',
      changeOrigin: true,
      rewrite: (path) => path.replace(/^\/api/, ''),
    },
  },
}
```

**Production Notes**:
- Production uses direct API URL via `VITE_API_URL`
- Backend CORS configuration required for production

---

### RQ-07: TypeScript Configuration

**Decision**: Strict mode with path aliases

**Rationale**:
- Catch type errors at compile time
- Better IDE support and autocomplete
- Path aliases (`@/`) for cleaner imports

**Key Settings**:
```json
{
  "compilerOptions": {
    "strict": true,
    "noUncheckedIndexedAccess": true,
    "baseUrl": ".",
    "paths": { "@/*": ["./src/*"] }
  }
}
```

---

## Technology Versions (Verified December 2025)

| Package | Version | Notes |
|---------|---------|-------|
| React | 18.3.1 | Latest stable |
| Vite | 6.0.1 | Latest major |
| TypeScript | 5.6.3 | Latest stable |
| Tailwind CSS | 3.4.15 | Latest v3 |
| @reduxjs/toolkit | 2.3.0 | RTK Query included |
| zustand | 5.0.0 | Latest major |
| react-router-dom | 6.28.0 | Latest v6 |
| lucide-react | 0.460.0 | Latest |

---

## Constitution Compliance Check

| Rule | Status | Notes |
|------|--------|-------|
| Phase II Technology Stack | COMPLIANT | React/Vite allowed |
| TypeScript Strict Mode | COMPLIANT | Enabled in config |
| Testing Requirements | PENDING | Tests in future phase |
| Clean Architecture | COMPLIANT | Clear layer separation |
| Spec-Driven Development | COMPLIANT | Spec exists |

---

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| localStorage XSS | High | CSP headers, input sanitization |
| Bundle size growth | Medium | Tree-shaking, code splitting |
| CORS in production | Medium | Backend CORS config |
| Theme flash | Low | Inline script initialization |

---

## References

- [Vite Documentation](https://vitejs.dev/)
- [RTK Query Guide](https://redux-toolkit.js.org/rtk-query/overview)
- [Zustand Documentation](https://docs.pmnd.rs/zustand)
- [shadcn/ui Components](https://ui.shadcn.com/)
- [Tailwind CSS Dark Mode](https://tailwindcss.com/docs/dark-mode)
