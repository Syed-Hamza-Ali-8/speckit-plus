import { useState, useCallback, useMemo } from 'react';
import { Plus } from 'lucide-react';
import { toast } from 'sonner';
import { Button } from '@/components/ui/button';
import { useAuth } from '@/hooks/useAuth';

// T077: Import all task components
import { TaskList } from '@/components/tasks/TaskList';
import { TaskFilters } from '@/components/tasks/TaskFilters';
import { TaskFormModal } from '@/components/tasks/TaskFormModal';
import { DeleteTaskDialog } from '@/components/tasks/DeleteTaskDialog';

// T078: Import useTaskFilters hook
import { useTaskFilters } from '@/hooks/useTaskFilters';

// T079: Import RTK Query hooks
import { useGetTasksQuery, useUpdateTaskMutation } from '@/services/taskApi';
import type { Task, TaskStatus } from '@/types/task';

/**
 * TasksPage
 *
 * Main page for task CRUD operations.
 * Orchestrates all task components:
 * - TaskFilters: Status and search filtering
 * - TaskList: Responsive task display (cards/table)
 * - TaskFormModal: Create and edit tasks
 * - DeleteTaskDialog: Confirm task deletion
 */
export function TasksPage() {
  const { logout } = useAuth();

  // T078: Filter state with URL sync
  const {
    filters,
    setStatus,
    setSearch,
    clearFilters,
    hasActiveFilters,
    debouncedSearch,
  } = useTaskFilters();

  // T079: Fetch tasks with filter params
  const queryParams = useMemo(() => {
    const params: { status?: TaskStatus } = {};
    if (filters.status !== 'all') {
      params.status = filters.status;
    }
    return params;
  }, [filters.status]);

  const { data: tasksData, isLoading, isFetching } = useGetTasksQuery(queryParams);

  // T091: Update task mutation for status toggle
  const [updateTask] = useUpdateTaskMutation();

  // T080: Create modal state
  const [isCreateModalOpen, setIsCreateModalOpen] = useState(false);

  // T081: Edit modal state (open + selected task)
  const [editModalTask, setEditModalTask] = useState<Task | null>(null);
  const isEditModalOpen = editModalTask !== null;

  // T082: Delete dialog state (open + selected task)
  const [deleteDialogTask, setDeleteDialogTask] = useState<Task | null>(null);
  const isDeleteDialogOpen = deleteDialogTask !== null;

  // Client-side search filtering (debouncedSearch from hook)
  const filteredTasks = useMemo(() => {
    const tasks = tasksData?.items ?? [];
    if (!debouncedSearch) return tasks;

    const searchLower = debouncedSearch.toLowerCase();
    return tasks.filter(
      (task) =>
        task.title.toLowerCase().includes(searchLower) ||
        (task.description && task.description.toLowerCase().includes(searchLower))
    );
  }, [tasksData?.items, debouncedSearch]);

  // Result counts for filter display
  const resultCount = useMemo(
    () => ({
      showing: filteredTasks.length,
      total: tasksData?.items.length ?? 0,
    }),
    [filteredTasks.length, tasksData?.items.length]
  );

  // T089: Handle edit callback
  const handleEdit = useCallback((task: Task) => {
    setEditModalTask(task);
  }, []);

  // T090: Handle delete callback
  const handleDelete = useCallback((task: Task) => {
    setDeleteDialogTask(task);
  }, []);

  // T091: Handle status toggle
  const handleToggleStatus = useCallback(
    async (task: Task) => {
      const newStatus: TaskStatus = task.status === 'completed' ? 'pending' : 'completed';
      try {
        await updateTask({
          id: task.id,
          data: { status: newStatus },
        }).unwrap();
        toast.success(`Task marked as ${newStatus}`);
      } catch (error) {
        const errorMessage = error instanceof Error ? error.message : 'Failed to update task';
        toast.error(errorMessage);
      }
    },
    [updateTask]
  );

  // Modal close handlers
  const handleCloseCreateModal = useCallback(() => {
    setIsCreateModalOpen(false);
  }, []);

  const handleCloseEditModal = useCallback(() => {
    setEditModalTask(null);
  }, []);

  const handleCloseDeleteDialog = useCallback(() => {
    setDeleteDialogTask(null);
  }, []);

  // Success handlers (just close modals - RTK Query handles cache invalidation)
  const handleCreateSuccess = useCallback(() => {
    // Modal closes itself via onClose
  }, []);

  const handleEditSuccess = useCallback(() => {
    // Modal closes itself via onClose
  }, []);

  const handleDeleteSuccess = useCallback(() => {
    // Dialog closes itself via onClose
  }, []);

  return (
    <div className="min-h-screen p-4 sm:p-6 lg:p-8">
      <div className="max-w-6xl mx-auto space-y-6">
        {/* T092: Page header with title and actions */}
        <div className="flex flex-col gap-4 sm:flex-row sm:items-center sm:justify-between">
          <h1 className="text-2xl font-bold">Tasks</h1>
          <div className="flex items-center gap-2">
            {/* T083: New Task button in header */}
            <Button onClick={() => setIsCreateModalOpen(true)}>
              <Plus className="h-4 w-4 mr-2" />
              New Task
            </Button>
            <Button variant="outline" onClick={logout}>
              Logout
            </Button>
          </div>
        </div>

        {/* T084: TaskFilters component */}
        <TaskFilters
          status={filters.status}
          search={filters.search}
          onStatusChange={setStatus}
          onSearchChange={setSearch}
          onClear={clearFilters}
          resultCount={resultCount}
          hasActiveFilters={hasActiveFilters}
        />

        {/* T085: TaskList component */}
        <TaskList
          tasks={filteredTasks}
          isLoading={isLoading || isFetching}
          onEdit={handleEdit}
          onDelete={handleDelete}
          onToggleStatus={handleToggleStatus}
          onCreateClick={() => setIsCreateModalOpen(true)}
          onClearFilters={clearFilters}
          hasFilters={hasActiveFilters}
        />

        {/* T086: TaskFormModal for create (no task prop) */}
        <TaskFormModal
          open={isCreateModalOpen}
          onClose={handleCloseCreateModal}
          onSuccess={handleCreateSuccess}
        />

        {/* T087: TaskFormModal for edit (with task prop) */}
        <TaskFormModal
          open={isEditModalOpen}
          onClose={handleCloseEditModal}
          task={editModalTask ?? undefined}
          onSuccess={handleEditSuccess}
        />

        {/* T088: DeleteTaskDialog */}
        <DeleteTaskDialog
          open={isDeleteDialogOpen}
          onClose={handleCloseDeleteDialog}
          task={deleteDialogTask}
          onSuccess={handleDeleteSuccess}
        />
      </div>
    </div>
  );
}
