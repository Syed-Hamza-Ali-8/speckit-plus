# Specification: Phase 3 Part 3 - Task CRUD UI

**Feature**: Complete task management interface with create, read, update, delete operations
**Date**: 2025-12-15
**Prerequisites**: Phase 3 Part 2 (Authentication UI) complete
**Output**: Fully functional task list with CRUD operations integrated with backend API

---

## Clarifications

### Session 2025-12-15

- Q: Create Task button placement? → A: "New Task" button in header bar (top-right of task list section)

---

## 1. Requirements

### 1.1 TaskList Layout (Responsive)

| Breakpoint | Layout | Description |
|------------|--------|-------------|
| Mobile (<640px) | Cards | Stacked task cards with title, status badge, actions |
| Tablet (640-1024px) | Cards Grid | 2-column card grid |
| Desktop (>1024px) | Table | Table with columns: Checkbox, Title, Status, Created, Actions |

**Card Layout Features:**
- Task title (truncated if long)
- Status badge (Pending/Completed)
- Created date
- Action buttons (Edit, Delete)
- Checkbox for status toggle

**Table Layout Features:**
- Sortable columns (Title, Status, Created)
- Bulk selection checkbox
- Inline status toggle
- Action dropdown (Edit, Delete)

### 1.2 Filters (Real-time)

| Filter | Type | Behavior |
|--------|------|----------|
| Status | Dropdown | Options: All, Pending, Completed |
| Search | Text input | Filter by title (debounced 300ms) |

**Filter Behavior:**
- Filters apply as user types/selects (no submit button)
- URL sync: `/tasks?status=pending&search=meeting`
- Clear all filters button
- Show result count: "Showing 5 of 12 tasks"

### 1.3 Create Task (Modal)

| Field | Type | Validation |
|-------|------|------------|
| Title | Text input | Required, max 100 chars |
| Description | Textarea | Optional, max 500 chars |

**Button Placement:**
- "New Task" button positioned in header bar (top-right of task list section)
- Consistent position across all breakpoints
- Always visible without scrolling

**Modal Behavior:**
- Opens via "New Task" button in header
- Closes on: Escape key, outside click, Cancel button, successful submit
- Form reset on close
- Loading state during submission
- Success toast: "Task created!"
- Error toast: API error message

### 1.4 Edit Task (Modal)

| Field | Type | Validation |
|-------|------|------------|
| Title | Text input | Required, max 100 chars |
| Description | Textarea | Optional, max 500 chars |
| Status | Toggle/Checkbox | Pending/Completed |

**Edit Modal Behavior:**
- Pre-populated with existing task data
- Same close behavior as Create
- Success toast: "Task updated!"
- Optimistic UI update (revert on error)

### 1.5 Delete Task (Dialog Confirmation)

| Element | Content |
|---------|---------|
| Title | "Delete Task" |
| Message | "Are you sure you want to delete '{task.title}'? This action cannot be undone." |
| Cancel | "Cancel" button (secondary) |
| Confirm | "Delete" button (destructive/red) |

**Delete Behavior:**
- Confirmation required before delete
- Loading state on Delete button
- Success toast: "Task deleted"
- Remove from list immediately (optimistic)
- Revert if API fails

### 1.6 Status Toggle

| Action | Result |
|--------|--------|
| Click checkbox | Toggle pending ↔ completed |
| API call | PATCH /tasks/{id} with new status |
| UI update | Optimistic, revert on error |

**Toggle Behavior:**
- Inline toggle (no confirmation)
- Visual feedback (checkbox animation)
- Toast on error only

### 1.7 Empty State

| Element | Content |
|---------|---------|
| Icon | ClipboardList or similar from Lucide |
| Heading | "No tasks yet" |
| Subtext | "Create your first task to get started" |
| CTA | "Create Task" button (opens modal) |

**Filtered Empty State:**
- Different message when filters active
- "No tasks match your filters"
- "Clear filters" link

---

## 2. Components

### 2.1 TaskList Component

**File**: `src/components/tasks/TaskList.tsx`

**Props**:
```typescript
interface TaskListProps {
  tasks: Task[];
  isLoading: boolean;
  onEdit: (task: Task) => void;
  onDelete: (task: Task) => void;
  onToggleStatus: (task: Task) => void;
}
```

**Features**:
- Responsive layout (cards/table)
- Loading skeleton
- Empty state handling

### 2.2 TaskCard Component

**File**: `src/components/tasks/TaskCard.tsx`

**Props**:
```typescript
interface TaskCardProps {
  task: Task;
  onEdit: () => void;
  onDelete: () => void;
  onToggleStatus: () => void;
}
```

### 2.3 TaskTable Component

**File**: `src/components/tasks/TaskTable.tsx`

**Props**:
```typescript
interface TaskTableProps {
  tasks: Task[];
  onEdit: (task: Task) => void;
  onDelete: (task: Task) => void;
  onToggleStatus: (task: Task) => void;
}
```

### 2.4 TaskFilters Component

**File**: `src/components/tasks/TaskFilters.tsx`

**Props**:
```typescript
interface TaskFiltersProps {
  status: 'all' | 'pending' | 'completed';
  search: string;
  onStatusChange: (status: string) => void;
  onSearchChange: (search: string) => void;
  onClear: () => void;
  resultCount: { showing: number; total: number };
}
```

### 2.5 TaskFormModal Component

**File**: `src/components/tasks/TaskFormModal.tsx`

**Props**:
```typescript
interface TaskFormModalProps {
  open: boolean;
  onClose: () => void;
  task?: Task; // If provided, edit mode
  onSuccess: () => void;
}
```

### 2.6 DeleteTaskDialog Component

**File**: `src/components/tasks/DeleteTaskDialog.tsx`

**Props**:
```typescript
interface DeleteTaskDialogProps {
  open: boolean;
  onClose: () => void;
  task: Task | null;
  onConfirm: () => void;
  isDeleting: boolean;
}
```

### 2.7 EmptyState Component

**File**: `src/components/tasks/EmptyState.tsx`

**Props**:
```typescript
interface EmptyStateProps {
  hasFilters: boolean;
  onCreateClick: () => void;
  onClearFilters: () => void;
}
```

### 2.8 Updated TasksPage

**File**: `src/pages/TasksPage.tsx`

**Features**:
- Orchestrates all task components
- Manages modal/dialog state
- Handles RTK Query hooks
- URL search params sync

---

## 3. API Integration

### 3.1 RTK Query Endpoints (existing)

```typescript
// Already defined in taskApi.ts
useGetTasksQuery()      // GET /tasks
useCreateTaskMutation() // POST /tasks
useUpdateTaskMutation() // PUT /tasks/{id}
useDeleteTaskMutation() // DELETE /tasks/{id}
```

### 3.2 Query Parameters

```typescript
interface GetTasksParams {
  status?: 'pending' | 'completed';
  search?: string;
}
```

**Note**: Backend filtering may need to be added if not already supported.

---

## 4. Flow Diagrams

### 4.1 Create Task Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                      CREATE TASK FLOW                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  1. User clicks "New Task" button in header                     │
│                    │                                             │
│                    ▼                                             │
│  2. TaskFormModal opens (empty form)                            │
│                    │                                             │
│                    ▼                                             │
│  3. User fills Title (required) + Description (optional)        │
│                    │                                             │
│                    ▼                                             │
│  4. User clicks "Create" → Form validates                       │
│                    │                                             │
│          ┌────────┴────────┐                                    │
│          │                 │                                     │
│          ▼                 ▼                                     │
│   Validation FAIL    Validation PASS                            │
│          │                 │                                     │
│          ▼                 ▼                                     │
│   Show inline         5. useCreateTaskMutation()                │
│   errors                   │                                     │
│                           │                                     │
│                  ┌────────┴────────┐                            │
│                  │                 │                             │
│                  ▼                 ▼                             │
│           API SUCCESS        API FAILURE                        │
│                  │                 │                             │
│                  ▼                 ▼                             │
│   6. Close modal         6. Show error toast                    │
│   7. Success toast       7. Keep modal open                     │
│   8. Invalidate cache                                           │
│      (auto-refetch)                                             │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 4.2 Delete Task Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                      DELETE TASK FLOW                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  1. User clicks Delete button on task                           │
│                    │                                             │
│                    ▼                                             │
│  2. DeleteTaskDialog opens                                      │
│     "Delete '{task.title}'?"                                    │
│                    │                                             │
│          ┌────────┴────────┐                                    │
│          │                 │                                     │
│          ▼                 ▼                                     │
│     Cancel            Confirm Delete                            │
│          │                 │                                     │
│          ▼                 ▼                                     │
│   Close dialog       3. useDeleteTaskMutation()                 │
│                           │                                     │
│                  ┌────────┴────────┐                            │
│                  │                 │                             │
│                  ▼                 ▼                             │
│           API SUCCESS        API FAILURE                        │
│                  │                 │                             │
│                  ▼                 ▼                             │
│   4. Close dialog        4. Show error toast                    │
│   5. Success toast       5. Keep dialog open                    │
│   6. Invalidate cache                                           │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 5. Success Criteria

### 5.1 Functional Requirements

| # | Criterion | Verification |
|---|-----------|--------------|
| 1 | Task list displays all user tasks | Visit `/tasks`, see tasks from API |
| 2 | Responsive layout works | Mobile: cards, Desktop: table |
| 3 | Status filter works | Select "Pending" → only pending tasks shown |
| 4 | Search filter works | Type "meeting" → matching tasks shown |
| 5 | Create task modal opens | Click "New Task" in header → modal appears |
| 6 | Create task saves to API | Fill form, submit → task in list |
| 7 | Edit task modal opens | Click Edit → modal with task data |
| 8 | Edit task saves to API | Change title, submit → updated in list |
| 9 | Delete confirmation shows | Click Delete → dialog appears |
| 10 | Delete removes task | Confirm delete → task removed from list |
| 11 | Status toggle works | Click checkbox → status changes |
| 12 | Empty state shows | Delete all tasks → empty state displayed |

### 5.2 UI/UX Requirements

| # | Criterion | Verification |
|---|-----------|--------------|
| 13 | Loading skeleton shows | Slow network → skeleton visible |
| 14 | Toast notifications work | CRUD actions → appropriate toasts |
| 15 | Form validation works | Submit empty → errors shown |
| 16 | Modal closes correctly | Escape/outside click → modal closes |
| 17 | Optimistic updates | Toggle status → instant UI update |

### 5.3 Technical Requirements

| # | Criterion | Verification |
|---|-----------|--------------|
| 18 | TypeScript compiles | `npx tsc --noEmit` passes |
| 19 | Production build succeeds | `npm run build` completes |
| 20 | RTK Query cache invalidation | Create/update/delete → list refreshes |

---

## 6. Dependencies

### 6.1 New shadcn/ui Components Needed

```bash
npx shadcn@latest add dialog
npx shadcn@latest add select
npx shadcn@latest add checkbox
npx shadcn@latest add textarea
npx shadcn@latest add skeleton
npx shadcn@latest add dropdown-menu
npx shadcn@latest add table
npx shadcn@latest add badge
```

### 6.2 Existing Dependencies (from Part 1 & 2)

- RTK Query (taskApi.ts with CRUD endpoints)
- React Hook Form + Zod
- Sonner (toast notifications)
- Lucide React (icons)
- Tailwind CSS

---

## 7. File Structure

```
src/
├── components/
│   └── tasks/
│       ├── TaskList.tsx           # NEW - Responsive list wrapper
│       ├── TaskCard.tsx           # NEW - Mobile card component
│       ├── TaskTable.tsx          # NEW - Desktop table component
│       ├── TaskFilters.tsx        # NEW - Status + search filters
│       ├── TaskFormModal.tsx      # NEW - Create/Edit modal
│       ├── DeleteTaskDialog.tsx   # NEW - Delete confirmation
│       └── EmptyState.tsx         # NEW - No tasks state
├── lib/
│   └── validations/
│       └── task.ts                # NEW - Zod schemas for task form
├── pages/
│   └── TasksPage.tsx              # UPDATED - Full CRUD UI
└── hooks/
    └── useTaskFilters.ts          # NEW - URL sync for filters
```

---

## 8. Out of Scope

- Drag-and-drop task reordering
- Task due dates / deadlines
- Task priority levels
- Task categories / tags
- Bulk actions (multi-select delete)
- Task attachments
- Task comments
- Pagination (all tasks loaded)

---

## 9. Q&A Summary

| # | Question | Answer |
|---|----------|--------|
| Q1 | TaskList layout | **Responsive**: Cards on mobile, Table on desktop |
| Q2 | Filters behavior | **Real-time**: Debounced search + status dropdown |
| Q3 | Create task location | **Modal**: Dialog overlay on /tasks page |
| Q4 | Delete confirmation | **Dialog**: Confirmation dialog with Cancel/Delete |
| Q5 | Empty state | **Illustration + CTA**: Icon, message, and Create button |
| Q6 | Create button placement | **Header bar**: Top-right of task list section |

**Date confirmed**: 2025-12-15
