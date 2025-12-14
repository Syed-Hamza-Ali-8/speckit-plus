# Tasks: Phase 3 Part 3 - Task CRUD UI

**Input**: Design documents from `/specs/phase3/part3-task-crud-ui/`
**Prerequisites**: spec.md, plan.md
**Tests**: Manual verification via checkpoints (no automated tests requested)

**Organization**: Tasks organized by user-provided outline (10 high-level tasks → 50+ granular tasks)

## Format: `[ID] [P?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- Include exact file paths in descriptions
- Checkpoints verify each phase completion

## Path Conventions

- **Frontend**: `phase2/frontend/`
- **Source**: `phase2/frontend/src/`

---

## Phase 1: shadcn/ui Components + Zod Schema

**Purpose**: Install required UI components and create task validation schema

**Checkpoint**: `npx tsc --noEmit` passes, all shadcn components in ui/ folder

- [X] T001 Install shadcn dialog component: `npx shadcn@latest add dialog` in `phase2/frontend/`
- [X] T002 Install shadcn select component: `npx shadcn@latest add select` in `phase2/frontend/`
- [X] T003 Install shadcn checkbox component: `npx shadcn@latest add checkbox` in `phase2/frontend/`
- [X] T004 Install shadcn textarea component: `npx shadcn@latest add textarea` in `phase2/frontend/`
- [X] T005 Install shadcn skeleton component: `npx shadcn@latest add skeleton` in `phase2/frontend/`
- [X] T006 Install shadcn dropdown-menu component: `npx shadcn@latest add dropdown-menu` in `phase2/frontend/`
- [X] T007 Install shadcn table component: `npx shadcn@latest add table` in `phase2/frontend/`
- [X] T008 Install shadcn badge component: `npx shadcn@latest add badge` in `phase2/frontend/`
- [X] T009 Create `phase2/frontend/src/lib/validations/task.ts` with taskSchema (title required max 100, description optional max 500)
- [X] T010 Export TypeScript types (TaskFormData) from task schema in `phase2/frontend/src/lib/validations/task.ts`
- [X] T011 Verify TypeScript compiles: `npx tsc --noEmit` passes

**Deliverables**:
- 8 new shadcn/ui components
- Task Zod validation schema

---

## Phase 2: TaskCard Component

**Purpose**: Create mobile task card display component

**Checkpoint**: TaskCard renders with mock task data showing all elements

- [X] T012 Create `phase2/frontend/src/components/tasks/TaskCard.tsx` with base structure and props interface
- [X] T013 Add task title display (truncated with ellipsis if > 50 chars) in `phase2/frontend/src/components/tasks/TaskCard.tsx`
- [X] T014 Add task description display (truncated, optional) in `phase2/frontend/src/components/tasks/TaskCard.tsx`
- [X] T015 Add status Badge component (Pending=yellow, Completed=green) in `phase2/frontend/src/components/tasks/TaskCard.tsx`
- [X] T016 Add created_at date display (formatted: "Dec 15, 2025") in `phase2/frontend/src/components/tasks/TaskCard.tsx`
- [X] T017 Add Checkbox for status toggle with onToggleStatus callback in `phase2/frontend/src/components/tasks/TaskCard.tsx`
- [X] T018 Add Edit button (Pencil icon) with onEdit callback in `phase2/frontend/src/components/tasks/TaskCard.tsx`
- [X] T019 Add Delete button (Trash icon) with onDelete callback in `phase2/frontend/src/components/tasks/TaskCard.tsx`
- [X] T020 Style TaskCard with Card component, proper spacing and dark mode support in `phase2/frontend/src/components/tasks/TaskCard.tsx`

**Deliverables**:
- TaskCard component with all UI elements
- Callbacks for edit, delete, toggle status

---

## Phase 3: EmptyState Component

**Purpose**: Create empty state with illustration and CTA

**Checkpoint**: EmptyState renders "No tasks yet" with Create button

- [X] T021 Create `phase2/frontend/src/components/tasks/EmptyState.tsx` with base structure and props interface
- [X] T022 Add ClipboardList icon from lucide-react centered in `phase2/frontend/src/components/tasks/EmptyState.tsx`
- [X] T023 Add "No tasks yet" heading in `phase2/frontend/src/components/tasks/EmptyState.tsx`
- [X] T024 Add "Create your first task to get started" subtext in `phase2/frontend/src/components/tasks/EmptyState.tsx`
- [X] T025 Add "Create Task" primary button with onCreateClick callback in `phase2/frontend/src/components/tasks/EmptyState.tsx`
- [X] T026 Add filtered empty state variant ("No tasks match your filters" + "Clear filters" link) in `phase2/frontend/src/components/tasks/EmptyState.tsx`

**Deliverables**:
- EmptyState component with two variants
- CTA button integration

---

## Phase 4: TaskTable Component

**Purpose**: Create desktop table view for tasks

**Checkpoint**: TaskTable renders tasks in table format with all columns

- [X] T027 Create `phase2/frontend/src/components/tasks/TaskTable.tsx` with base structure and props interface
- [X] T028 Add Table structure with TableHeader, TableBody from shadcn in `phase2/frontend/src/components/tasks/TaskTable.tsx`
- [X] T029 Add Checkbox column for status toggle in `phase2/frontend/src/components/tasks/TaskTable.tsx`
- [X] T030 Add Title column (truncated if > 40 chars) in `phase2/frontend/src/components/tasks/TaskTable.tsx`
- [X] T031 Add Description column (truncated, show "—" if null) in `phase2/frontend/src/components/tasks/TaskTable.tsx`
- [X] T032 Add Status column with Badge component in `phase2/frontend/src/components/tasks/TaskTable.tsx`
- [X] T033 Add Created column with formatted date in `phase2/frontend/src/components/tasks/TaskTable.tsx`
- [X] T034 Add Actions column with DropdownMenu (Edit, Delete options) in `phase2/frontend/src/components/tasks/TaskTable.tsx`
- [X] T035 Wire up callbacks: onEdit, onDelete, onToggleStatus in `phase2/frontend/src/components/tasks/TaskTable.tsx`

**Deliverables**:
- TaskTable component with all columns
- Action dropdown menu

---

## Phase 5: TaskFilters + useTaskFilters Hook

**Purpose**: Implement filtering with URL synchronization

**Checkpoint**: Filters update URL params and filter results

- [X] T036 Create `phase2/frontend/src/hooks/useTaskFilters.ts` with base hook structure
- [X] T037 Add status filter state (all/pending/completed) with URL sync in `phase2/frontend/src/hooks/useTaskFilters.ts`
- [X] T038 Add search filter state with URL sync in `phase2/frontend/src/hooks/useTaskFilters.ts`
- [X] T039 Add debounce (300ms) for search input in `phase2/frontend/src/hooks/useTaskFilters.ts`
- [X] T040 Add clearFilters function in `phase2/frontend/src/hooks/useTaskFilters.ts`
- [X] T041 Create `phase2/frontend/src/components/tasks/TaskFilters.tsx` with base structure
- [X] T042 Add status Select dropdown (All, Pending, Completed) in `phase2/frontend/src/components/tasks/TaskFilters.tsx`
- [X] T043 Add search Input with placeholder "Search tasks..." in `phase2/frontend/src/components/tasks/TaskFilters.tsx`
- [X] T044 Add result count display "Showing X of Y tasks" in `phase2/frontend/src/components/tasks/TaskFilters.tsx`
- [X] T045 Add "Clear filters" button (shown when filters active) in `phase2/frontend/src/components/tasks/TaskFilters.tsx`

**Deliverables**:
- useTaskFilters hook with URL sync
- TaskFilters component

---

## Phase 6: TaskList + Responsive Layout

**Purpose**: Create responsive list wrapper with loading states

**Checkpoint**: Mobile shows cards, Desktop shows table

- [X] T046 Create `phase2/frontend/src/components/tasks/TaskList.tsx` with base structure and props interface
- [X] T047 Add responsive breakpoint detection using Tailwind classes in `phase2/frontend/src/components/tasks/TaskList.tsx`
- [X] T048 Render TaskCard components for mobile (<640px) in vertical stack in `phase2/frontend/src/components/tasks/TaskList.tsx`
- [X] T049 Render TaskCard components for tablet (640-1024px) in 2-column grid in `phase2/frontend/src/components/tasks/TaskList.tsx`
- [X] T050 Render TaskTable component for desktop (>1024px) in `phase2/frontend/src/components/tasks/TaskList.tsx`
- [X] T051 Add loading Skeleton state (3-5 skeleton cards/rows) in `phase2/frontend/src/components/tasks/TaskList.tsx`
- [X] T052 Integrate EmptyState when tasks array is empty in `phase2/frontend/src/components/tasks/TaskList.tsx`

**Deliverables**:
- TaskList responsive wrapper
- Loading skeleton integration

---

## Phase 7: TaskFormModal (Create/Edit)

**Purpose**: Implement create and edit task modal

**Checkpoint**: Create new task, edit existing task, both save to API

- [X] T053 Create `phase2/frontend/src/components/tasks/TaskFormModal.tsx` with Dialog wrapper
- [X] T054 Add React Hook Form setup with Zod resolver (taskSchema) in `phase2/frontend/src/components/tasks/TaskFormModal.tsx`
- [X] T055 Add Title Input field (required, max 100 chars) with validation in `phase2/frontend/src/components/tasks/TaskFormModal.tsx`
- [X] T056 Add Description Textarea field (optional, max 500 chars) in `phase2/frontend/src/components/tasks/TaskFormModal.tsx`
- [X] T057 Add Status Checkbox (edit mode only, toggle pending/completed) in `phase2/frontend/src/components/tasks/TaskFormModal.tsx`
- [X] T058 Add conditional title: "New Task" vs "Edit Task" based on mode in `phase2/frontend/src/components/tasks/TaskFormModal.tsx`
- [X] T059 Pre-populate form with task data in edit mode in `phase2/frontend/src/components/tasks/TaskFormModal.tsx`
- [X] T060 Integrate useCreateTaskMutation for create mode in `phase2/frontend/src/components/tasks/TaskFormModal.tsx`
- [X] T061 Integrate useUpdateTaskMutation for edit mode in `phase2/frontend/src/components/tasks/TaskFormModal.tsx`
- [X] T062 Add LoadingButton with loading state during submission in `phase2/frontend/src/components/tasks/TaskFormModal.tsx`
- [X] T063 Add success toast ("Task created!" / "Task updated!") on success in `phase2/frontend/src/components/tasks/TaskFormModal.tsx`
- [X] T064 Add error toast with API error message on failure in `phase2/frontend/src/components/tasks/TaskFormModal.tsx`
- [X] T065 Close modal on: successful submit, Cancel button, Escape key, outside click in `phase2/frontend/src/components/tasks/TaskFormModal.tsx`
- [X] T066 Reset form on modal close in `phase2/frontend/src/components/tasks/TaskFormModal.tsx`

**Deliverables**:
- TaskFormModal component (create + edit modes)
- RTK Query mutation integration

---

## Phase 8: DeleteTaskDialog

**Purpose**: Implement delete confirmation dialog

**Checkpoint**: Delete button shows dialog, confirm deletes task

- [X] T067 Create `phase2/frontend/src/components/tasks/DeleteTaskDialog.tsx` with Dialog wrapper
- [X] T068 Add dialog title "Delete Task" in `phase2/frontend/src/components/tasks/DeleteTaskDialog.tsx`
- [X] T069 Add confirmation message with task title interpolated in `phase2/frontend/src/components/tasks/DeleteTaskDialog.tsx`
- [X] T070 Add Cancel button (secondary variant) in `phase2/frontend/src/components/tasks/DeleteTaskDialog.tsx`
- [X] T071 Add Delete button (destructive variant, red) in `phase2/frontend/src/components/tasks/DeleteTaskDialog.tsx`
- [X] T072 Integrate useDeleteTaskMutation in `phase2/frontend/src/components/tasks/DeleteTaskDialog.tsx`
- [X] T073 Add loading state on Delete button during API call in `phase2/frontend/src/components/tasks/DeleteTaskDialog.tsx`
- [X] T074 Add success toast "Task deleted" on success in `phase2/frontend/src/components/tasks/DeleteTaskDialog.tsx`
- [X] T075 Add error toast with API error message on failure in `phase2/frontend/src/components/tasks/DeleteTaskDialog.tsx`
- [X] T076 Close dialog on successful delete or Cancel in `phase2/frontend/src/components/tasks/DeleteTaskDialog.tsx`

**Deliverables**:
- DeleteTaskDialog component
- RTK Query delete mutation integration

---

## Phase 9: Updated TasksPage

**Purpose**: Orchestrate all components in the main page

**Checkpoint**: Full CRUD operations work through UI

- [X] T077 Update `phase2/frontend/src/pages/TasksPage.tsx` with new imports for all task components
- [X] T078 Add useTaskFilters hook integration in `phase2/frontend/src/pages/TasksPage.tsx`
- [X] T079 Add useGetTasksQuery with filter params in `phase2/frontend/src/pages/TasksPage.tsx`
- [X] T080 Add state for create modal open/close in `phase2/frontend/src/pages/TasksPage.tsx`
- [X] T081 Add state for edit modal (open + selected task) in `phase2/frontend/src/pages/TasksPage.tsx`
- [X] T082 Add state for delete dialog (open + selected task) in `phase2/frontend/src/pages/TasksPage.tsx`
- [X] T083 Add "New Task" button in page header in `phase2/frontend/src/pages/TasksPage.tsx`
- [X] T084 Render TaskFilters component with filter state in `phase2/frontend/src/pages/TasksPage.tsx`
- [X] T085 Render TaskList component with tasks and loading state in `phase2/frontend/src/pages/TasksPage.tsx`
- [X] T086 Render TaskFormModal for create (no task prop) in `phase2/frontend/src/pages/TasksPage.tsx`
- [X] T087 Render TaskFormModal for edit (with task prop) in `phase2/frontend/src/pages/TasksPage.tsx`
- [X] T088 Render DeleteTaskDialog with selected task in `phase2/frontend/src/pages/TasksPage.tsx`
- [X] T089 Wire handleEdit callback to open edit modal in `phase2/frontend/src/pages/TasksPage.tsx`
- [X] T090 Wire handleDelete callback to open delete dialog in `phase2/frontend/src/pages/TasksPage.tsx`
- [X] T091 Wire handleToggleStatus callback to useUpdateTaskMutation in `phase2/frontend/src/pages/TasksPage.tsx`
- [X] T092 Add page title "Tasks" and proper layout spacing in `phase2/frontend/src/pages/TasksPage.tsx`

**Deliverables**:
- Fully functional TasksPage
- All CRUD operations working

---

## Phase 10: Final Polish + Build Verification

**Purpose**: TypeScript verification and production build check

**Checkpoint**: `npm run build` → 0 errors, all CRUD flows functional

- [X] T093 Review and fix any TypeScript errors: `npx tsc --noEmit`
- [X] T094 Run production build: `npm run build` → verify dist/ created
- [ ] T095 Manual test: Create task flow (New Task → fill form → submit → appears in list)
- [ ] T096 Manual test: Edit task flow (click Edit → modify → submit → updates in list)
- [ ] T097 Manual test: Delete task flow (click Delete → confirm → removed from list)
- [ ] T098 Manual test: Status toggle (click checkbox → status changes)
- [ ] T099 Manual test: Filter by status (select Pending → only pending tasks shown)
- [ ] T100 Manual test: Search filter (type in search → matching tasks shown)
- [ ] T101 Manual test: Empty state (delete all tasks → "No tasks yet" shown)
- [ ] T102 Manual test: Responsive layout (resize browser → cards on mobile, table on desktop)

**Deliverables**:
- Zero TypeScript errors
- Production build successful
- All acceptance criteria verified

---

## Dependencies & Execution Order

### Phase Dependencies

```text
Phase 1: shadcn/ui + Zod Schema ──────────────────┐
                                                   │
                                                   ▼
Phase 2: TaskCard Component ──────────────────────┐
         │                                         │
         ▼                                         │
Phase 3: EmptyState Component ◄───────────────────┤
         │                                         │
         ▼                                         │
Phase 4: TaskTable Component ◄────────────────────┤
         │                                         │
         ▼                                         │
Phase 5: TaskFilters + useTaskFilters ────────────┤
         │                                         │
         ▼                                         │
Phase 6: TaskList (Responsive) ◄──────────────────┘
         │
         ▼
Phase 7: TaskFormModal (Create/Edit)
         │
         ▼
Phase 8: DeleteTaskDialog
         │
         ▼
Phase 9: Updated TasksPage ───────────────────────┐
         │                                         │
         ▼                                         │
Phase 10: Final Polish + Build ◄──────────────────┘
```

### Within Each Phase

- T001-T011: T001-T008 can run in parallel (different shadcn commands), T009-T011 sequential
- T012-T020: Sequential (same file)
- T021-T026: Sequential (same file)
- T027-T035: Sequential (same file)
- T036-T045: T036-T040 sequential, T041-T045 sequential (different files)
- T046-T052: Sequential (same file)
- T053-T066: Sequential (same component)
- T067-T076: Sequential (same component)
- T077-T092: Sequential (same page)
- T093-T102: Sequential (verification)

### Parallel Opportunities

```bash
# Phase 1 (shadcn installs can run in parallel):
T001-T008: Install shadcn components    [P]

# Phase 2-4 (after Phase 1, these components can be built in parallel):
T012-T020: TaskCard                     [P]
T021-T026: EmptyState                   [P]
T027-T035: TaskTable                    [P]
```

---

## Implementation Strategy

### Sequential Execution (Recommended)

1. Complete Phase 1 → Checkpoint: shadcn components installed
2. Complete Phase 2 → Checkpoint: TaskCard renders
3. Complete Phase 3 → Checkpoint: EmptyState renders
4. Complete Phase 4 → Checkpoint: TaskTable renders
5. Complete Phase 5 → Checkpoint: Filters work
6. Complete Phase 6 → Checkpoint: Responsive layout works
7. Complete Phase 7 → Checkpoint: Create/Edit modals work
8. Complete Phase 8 → Checkpoint: Delete dialog works
9. Complete Phase 9 → Checkpoint: Full CRUD on TasksPage
10. Complete Phase 10 → **DONE**

### MVP Validation Points

- **After Phase 6**: Task list displays with responsive layout
- **After Phase 9**: Full CRUD operations working
- **After Phase 10**: Production-ready Task CRUD UI

---

## Files Created/Modified Summary

| File | Phase | Tasks | Status |
|------|-------|-------|--------|
| `src/components/ui/dialog.tsx` | 1 | T001 | NEW (shadcn) |
| `src/components/ui/select.tsx` | 1 | T002 | NEW (shadcn) |
| `src/components/ui/checkbox.tsx` | 1 | T003 | NEW (shadcn) |
| `src/components/ui/textarea.tsx` | 1 | T004 | NEW (shadcn) |
| `src/components/ui/skeleton.tsx` | 1 | T005 | NEW (shadcn) |
| `src/components/ui/dropdown-menu.tsx` | 1 | T006 | NEW (shadcn) |
| `src/components/ui/table.tsx` | 1 | T007 | NEW (shadcn) |
| `src/components/ui/badge.tsx` | 1 | T008 | NEW (shadcn) |
| `src/lib/validations/task.ts` | 1 | T009-T010 | NEW |
| `src/components/tasks/TaskCard.tsx` | 2 | T012-T020 | NEW |
| `src/components/tasks/EmptyState.tsx` | 3 | T021-T026 | NEW |
| `src/components/tasks/TaskTable.tsx` | 4 | T027-T035 | NEW |
| `src/hooks/useTaskFilters.ts` | 5 | T036-T040 | NEW |
| `src/components/tasks/TaskFilters.tsx` | 5 | T041-T045 | NEW |
| `src/components/tasks/TaskList.tsx` | 6 | T046-T052 | NEW |
| `src/components/tasks/TaskFormModal.tsx` | 7 | T053-T066 | NEW |
| `src/components/tasks/DeleteTaskDialog.tsx` | 8 | T067-T076 | NEW |
| `src/pages/TasksPage.tsx` | 9 | T077-T092 | MODIFIED |

**Total**: 102 tasks, 18 files (17 new, 1 modified)

---

## Notes

- [P] tasks = different files, no dependencies
- Phases must complete in order (each depends on previous)
- Checkpoints verify each phase before proceeding
- No automated tests requested (manual verification only)
- Backend must be running for API integration testing
- RTK Query hooks already exist in taskApi.ts
