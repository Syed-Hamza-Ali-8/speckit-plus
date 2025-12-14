# Implementation Plan: Phase 3 Part 3 - Task CRUD UI

**Feature**: Complete task management interface with CRUD operations
**Prerequisites**: Phase 3 Part 2 (Authentication UI) complete
**Output**: Fully functional task list with responsive design, filters, and modals

---

## 1. Implementation Phases

### Phase 1: shadcn/ui Components + Zod Schema

**Purpose**: Install required UI components and create task validation schema

**Tasks**:
1. Install shadcn/ui components: dialog, select, checkbox, textarea, skeleton, dropdown-menu, table, badge
2. Create `src/lib/validations/task.ts` with Zod schema for task form
3. Verify TypeScript compilation passes

**Deliverables**:
- New shadcn/ui components in `src/components/ui/`
- `src/lib/validations/task.ts` with taskSchema

**Checkpoint**: `npx tsc --noEmit` passes

---

### Phase 2: Task Card + Empty State Components

**Purpose**: Build foundational display components

**Tasks**:
1. Create `src/components/tasks/TaskCard.tsx` - Mobile card component
   - Task title, status badge, created date
   - Checkbox for status toggle
   - Edit/Delete action buttons
2. Create `src/components/tasks/EmptyState.tsx` - No tasks state
   - ClipboardList icon
   - "No tasks yet" heading
   - "Create Task" CTA button
   - Filtered empty state variant

**Deliverables**:
- TaskCard component
- EmptyState component

**Checkpoint**: Components render correctly in isolation

---

### Phase 3: Task Table Component

**Purpose**: Build desktop table view

**Tasks**:
1. Create `src/components/tasks/TaskTable.tsx` - Desktop table component
   - Columns: Checkbox, Title, Description (truncated), Status, Created, Actions
   - Status badge styling (Pending=yellow, Completed=green)
   - Action dropdown (Edit, Delete)
   - Inline checkbox for status toggle

**Deliverables**:
- TaskTable component with all columns

**Checkpoint**: Table renders with sample data

---

### Phase 4: Task Filters + URL Sync Hook

**Purpose**: Implement filtering with URL synchronization

**Tasks**:
1. Create `src/hooks/useTaskFilters.ts` - Filter state management
   - Status filter (all/pending/completed)
   - Search filter (debounced 300ms)
   - URL sync: `/tasks?status=pending&search=meeting`
   - Clear filters function
2. Create `src/components/tasks/TaskFilters.tsx` - Filter UI
   - Status dropdown (Select component)
   - Search input with debounce
   - Result count: "Showing X of Y tasks"
   - Clear filters button

**Deliverables**:
- useTaskFilters hook
- TaskFilters component

**Checkpoint**: Filters update URL and filter works

---

### Phase 5: Task List + Responsive Layout

**Purpose**: Create responsive list wrapper

**Tasks**:
1. Create `src/components/tasks/TaskList.tsx` - Responsive list
   - Mobile (<640px): Render TaskCard in stack
   - Tablet (640-1024px): Render TaskCard in 2-col grid
   - Desktop (>1024px): Render TaskTable
   - Loading skeleton state
   - Empty state integration

**Deliverables**:
- TaskList component with responsive behavior

**Checkpoint**: Responsive breakpoints work correctly

---

### Phase 6: Task Form Modal (Create/Edit)

**Purpose**: Implement create and edit task modal

**Tasks**:
1. Create `src/components/tasks/TaskFormModal.tsx` - Form modal
   - React Hook Form + Zod validation
   - Title field (required, max 100 chars)
   - Description field (optional, max 500 chars)
   - Status toggle (edit mode only)
   - Loading state during submission
   - Success/error toast notifications
   - Close on: Escape, outside click, Cancel, successful submit
2. Integrate with RTK Query mutations
   - useCreateTaskMutation for new tasks
   - useUpdateTaskMutation for edits

**Deliverables**:
- TaskFormModal component (create + edit modes)

**Checkpoint**: Create and edit flows work end-to-end

---

### Phase 7: Delete Task Dialog

**Purpose**: Implement delete confirmation

**Tasks**:
1. Create `src/components/tasks/DeleteTaskDialog.tsx` - Confirmation dialog
   - Title: "Delete Task"
   - Message: "Are you sure you want to delete '{task.title}'?"
   - Cancel button (secondary)
   - Delete button (destructive/red)
   - Loading state during deletion
2. Integrate with useDeleteTaskMutation

**Deliverables**:
- DeleteTaskDialog component

**Checkpoint**: Delete flow with confirmation works

---

### Phase 8: Updated TasksPage

**Purpose**: Orchestrate all components in the main page

**Tasks**:
1. Update `src/pages/TasksPage.tsx` to:
   - Use useGetTasksQuery with filter params
   - Manage modal states (create, edit, delete)
   - Handle CRUD callbacks
   - Integrate TaskFilters, TaskList, TaskFormModal, DeleteTaskDialog
   - Add "New Task" button in header
2. Wire up all event handlers
3. Implement optimistic updates where applicable

**Deliverables**:
- Fully functional TasksPage

**Checkpoint**: All CRUD operations work through UI

---

### Phase 9: Final Polish + Build Verification

**Purpose**: TypeScript verification and production build

**Tasks**:
1. Review and fix any TypeScript errors: `npx tsc --noEmit`
2. Run production build: `npm run build`
3. Manual test: Create task flow
4. Manual test: Edit task flow
5. Manual test: Delete task flow
6. Manual test: Filter/search functionality
7. Manual test: Responsive layout (mobile/desktop)
8. Manual test: Empty state display

**Deliverables**:
- Zero TypeScript errors
- Production build successful
- All acceptance criteria verified

**Checkpoint**: `npm run build` succeeds, all manual tests pass

---

## 2. Dependency Graph

```
Phase 1: shadcn/ui + Zod Schema
         │
         ▼
Phase 2: TaskCard + EmptyState ─────────────────┐
         │                                       │
         ▼                                       │
Phase 3: TaskTable ──────────────────────────────┤
         │                                       │
         ▼                                       │
Phase 4: TaskFilters + useTaskFilters Hook ─────┤
         │                                       │
         ▼                                       │
Phase 5: TaskList (Responsive Wrapper) ◄────────┘
         │
         ▼
Phase 6: TaskFormModal (Create/Edit)
         │
         ▼
Phase 7: DeleteTaskDialog
         │
         ▼
Phase 8: Updated TasksPage ──────────────────────┐
         │                                       │
         ▼                                       │
Phase 9: Final Polish + Build ◄──────────────────┘
```

**Critical Path**: Phases 1-8 must execute sequentially

---

## 3. Complete File Structure

```
phase2/frontend/
├── src/
│   ├── components/
│   │   ├── tasks/
│   │   │   ├── TaskList.tsx           # NEW - Responsive list wrapper
│   │   │   ├── TaskCard.tsx           # NEW - Mobile card component
│   │   │   ├── TaskTable.tsx          # NEW - Desktop table component
│   │   │   ├── TaskFilters.tsx        # NEW - Status + search filters
│   │   │   ├── TaskFormModal.tsx      # NEW - Create/Edit modal
│   │   │   ├── DeleteTaskDialog.tsx   # NEW - Delete confirmation
│   │   │   └── EmptyState.tsx         # NEW - No tasks state
│   │   ├── ui/
│   │   │   ├── dialog.tsx             # NEW (shadcn)
│   │   │   ├── select.tsx             # NEW (shadcn)
│   │   │   ├── checkbox.tsx           # NEW (shadcn)
│   │   │   ├── textarea.tsx           # NEW (shadcn)
│   │   │   ├── skeleton.tsx           # NEW (shadcn)
│   │   │   ├── dropdown-menu.tsx      # NEW (shadcn)
│   │   │   ├── table.tsx              # NEW (shadcn)
│   │   │   └── badge.tsx              # NEW (shadcn)
│   │   ├── auth/                      # EXISTING
│   │   └── layout/                    # EXISTING
│   ├── hooks/
│   │   ├── useAuth.ts                 # EXISTING
│   │   ├── useTheme.ts                # EXISTING
│   │   ├── useReturnUrl.ts            # EXISTING
│   │   └── useTaskFilters.ts          # NEW - URL sync for filters
│   ├── lib/
│   │   ├── utils.ts                   # EXISTING
│   │   └── validations/
│   │       ├── auth.ts                # EXISTING
│   │       └── task.ts                # NEW - Zod schemas for task
│   ├── pages/
│   │   ├── LoginPage.tsx              # EXISTING
│   │   ├── RegisterPage.tsx           # EXISTING
│   │   └── TasksPage.tsx              # UPDATED - Full CRUD UI
│   ├── services/
│   │   ├── api.ts                     # EXISTING
│   │   ├── authApi.ts                 # EXISTING
│   │   └── taskApi.ts                 # EXISTING (hooks ready)
│   └── types/
│       ├── auth.ts                    # EXISTING
│       └── task.ts                    # EXISTING
└── package.json                       # UPDATED - shadcn deps
```

---

## 4. Success Criteria

| # | Criterion | Verification |
|---|-----------|--------------|
| 1 | shadcn/ui components installed | All components in `src/components/ui/` |
| 2 | Task Zod schema works | `npx tsc --noEmit` passes |
| 3 | TaskCard renders | Mobile view shows task cards |
| 4 | TaskTable renders | Desktop view shows table |
| 5 | EmptyState shows | No tasks → empty state displayed |
| 6 | Filters work | Status/search filter tasks |
| 7 | URL sync works | `/tasks?status=pending` filters correctly |
| 8 | Responsive layout | Mobile=cards, Desktop=table |
| 9 | Create modal works | New task → success toast → appears in list |
| 10 | Edit modal works | Edit task → success toast → updates in list |
| 11 | Delete dialog works | Confirm delete → task removed |
| 12 | Status toggle works | Checkbox → status changes |
| 13 | Loading skeleton | Shows during API calls |
| 14 | Toast notifications | All CRUD operations show toasts |
| 15 | TypeScript compiles | `npx tsc --noEmit` passes |
| 16 | Production build | `npm run build` succeeds |

---

## 5. Dependencies

### New Packages (via shadcn CLI)

```bash
# Run from phase2/frontend/
npx shadcn@latest add dialog
npx shadcn@latest add select
npx shadcn@latest add checkbox
npx shadcn@latest add textarea
npx shadcn@latest add skeleton
npx shadcn@latest add dropdown-menu
npx shadcn@latest add table
npx shadcn@latest add badge
```

### Existing Dependencies (from Part 1 & 2)

- `@reduxjs/toolkit` - RTK Query (taskApi hooks ready)
- `react-hook-form` + `@hookform/resolvers` + `zod` - Form validation
- `sonner` - Toast notifications
- `lucide-react` - Icons
- `tailwindcss` - Styling

---

## 6. Risk Mitigation

| Risk | Mitigation |
|------|------------|
| shadcn/ui install issues | Use `--overwrite` flag if conflicts |
| Responsive breakpoints | Test on actual devices/emulator |
| RTK Query cache stale | Use proper invalidatesTags (already set up) |
| Filter URL sync race conditions | Debounce filter changes (300ms) |
| Modal close on click outside | Use shadcn Dialog `onOpenChange` |

---

## 7. Constitution Compliance

| Rule | Status | Notes |
|------|--------|-------|
| Spec-Driven Development | COMPLIANT | spec.md exists |
| Phase III Technology Stack | COMPLIANT | React/Vite allowed per constitution |
| TypeScript Strict Mode | COMPLIANT | Enabled in tsconfig.json |
| Clean Architecture | COMPLIANT | Components, hooks, services separation |
| Testing Requirements | PENDING | Manual tests in Phase 9 |
| Interface Contracts | COMPLIANT | TypeScript interfaces defined |

---

## 8. Related Documents

- [spec.md](./spec.md) - Formal specification
- [Phase 3 Part 2 plan.md](../part2-auth-ui/plan.md) - Auth UI plan
- [task.ts](../../phase2/frontend/src/types/task.ts) - Task type definitions
- [taskApi.ts](../../phase2/frontend/src/services/taskApi.ts) - RTK Query endpoints
