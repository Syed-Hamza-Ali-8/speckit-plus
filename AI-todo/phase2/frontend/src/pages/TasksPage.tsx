import { useState, useCallback, useMemo } from 'react';
import { motion } from 'framer-motion';
import { Plus, Sparkles } from 'lucide-react';
import { toast } from 'sonner';
import { GlassButton } from '@/components/ui/GlassButton';
import { cn } from '@/lib/utils';

import { TaskList } from '@/components/tasks/TaskList';
import { TaskFilters } from '@/components/tasks/TaskFilters';
import { TaskFormModal } from '@/components/tasks/TaskFormModal';
import { DeleteTaskDialog } from '@/components/tasks/DeleteTaskDialog';
import { useTaskFilters } from '@/hooks/useTaskFilters';
import { useGetTasksQuery, useUpdateTaskMutation, useSetTaskPriorityMutation, useAddTaskTagsMutation, useRemoveTaskTagsMutation } from '@/services/taskApi';
import type { Task, TaskStatus } from '@/types/task';

export function TasksPage() {
  const {
    filters,
    setStatus,
    setPriority,
    setSearch,
    clearFilters,
    hasActiveFilters,
    debouncedSearch,
  } = useTaskFilters();

  const queryParams = useMemo(() => {
    const params: { status?: TaskStatus; priority?: 'low' | 'medium' | 'high' } = {};
    if (filters.status !== 'all') {
      params.status = filters.status;
    }
    if (filters.priority !== 'all') {
      params.priority = filters.priority;
    }
    return params;
  }, [filters.status, filters.priority]);

  const { data: tasksData, isLoading, isFetching } = useGetTasksQuery(queryParams);
  const [updateTask] = useUpdateTaskMutation();
  const [setTaskPriority] = useSetTaskPriorityMutation();
  const [addTaskTags] = useAddTaskTagsMutation();
  const [removeTaskTags] = useRemoveTaskTagsMutation();

  const [isCreateModalOpen, setIsCreateModalOpen] = useState(false);
  const [editModalTask, setEditModalTask] = useState<Task | null>(null);
  const isEditModalOpen = editModalTask !== null;
  const [deleteDialogTask, setDeleteDialogTask] = useState<Task | null>(null);
  const isDeleteDialogOpen = deleteDialogTask !== null;

  const filteredTasks = useMemo(() => {
    let tasks = tasksData?.items ?? [];

    // Apply priority filter
    if (filters.priority !== 'all') {
      tasks = tasks.filter(task => task.priority === filters.priority);
    }

    // Apply search filter
    if (debouncedSearch) {
      const searchLower = debouncedSearch.toLowerCase();
      tasks = tasks.filter(
        (task) =>
          task.title.toLowerCase().includes(searchLower) ||
          (task.description && task.description.toLowerCase().includes(searchLower)) ||
          (task.tags && task.tags.some(tag => tag.toLowerCase().includes(searchLower)))
      );
    }

    return tasks;
  }, [tasksData?.items, filters.priority, debouncedSearch]);

  const resultCount = useMemo(
    () => ({
      showing: filteredTasks.length,
      total: tasksData?.items.length ?? 0,
    }),
    [filteredTasks.length, tasksData?.items.length]
  );

  const handleEdit = useCallback((task: Task) => {
    setEditModalTask(task);
  }, []);

  const handleDelete = useCallback((task: Task) => {
    setDeleteDialogTask(task);
  }, []);

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

  const handleSetPriority = useCallback(
    async (task: Task, priority: 'low' | 'medium' | 'high') => {
      try {
        await setTaskPriority({
          id: task.id,
          priority: priority,
        }).unwrap();
        toast.success(`Task priority set to ${priority}`);
      } catch (error) {
        const errorMessage = error instanceof Error ? error.message : 'Failed to update task priority';
        toast.error(errorMessage);
      }
    },
    [setTaskPriority]
  );

  const handleSetTags = useCallback(
    async (task: Task, tags: string[]) => {
      // For now, we'll just open the edit modal since the task form already handles tags
      setEditModalTask(task);
    },
    []
  );

  const handleCloseCreateModal = useCallback(() => {
    setIsCreateModalOpen(false);
  }, []);

  const handleCloseEditModal = useCallback(() => {
    setEditModalTask(null);
  }, []);

  const handleCloseDeleteDialog = useCallback(() => {
    setDeleteDialogTask(null);
  }, []);

  const handleCreateSuccess = useCallback(() => {}, []);
  const handleEditSuccess = useCallback(() => {}, []);
  const handleDeleteSuccess = useCallback(() => {}, []);

  // Calculate stats
  const stats = useMemo(() => {
    const tasks = filteredTasks; // Use filtered tasks instead of all tasks
    const completed = tasks.filter((t) => t.status === 'completed').length;
    const pending = tasks.filter((t) => t.status === 'pending').length;
    return { total: tasks.length, completed, pending };
  }, [filteredTasks]);

  return (
    <div className="relative min-h-screen">
      {/* Background gradient orbs */}
      <div className="absolute inset-0 overflow-hidden pointer-events-none">
        <div
          className={cn(
            'absolute -top-40 -right-40 w-96 h-96',
            'bg-purple-500/20 dark:bg-purple-500/10',
            'rounded-full blur-3xl'
          )}
        />
        <div
          className={cn(
            'absolute bottom-0 -left-40 w-96 h-96',
            'bg-blue-500/20 dark:bg-blue-500/10',
            'rounded-full blur-3xl'
          )}
        />
      </div>

      <div className="relative p-4 sm:p-6 lg:p-8">
        <div className="max-w-6xl mx-auto space-y-6">
          {/* Page header */}
          <motion.div
            initial={{ opacity: 0, y: -20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.3 }}
            className="flex flex-col gap-4 sm:flex-row sm:items-center sm:justify-between"
          >
            <div>
              <h1 className="text-3xl font-bold">
                <span className="text-gradient">My Tasks</span>
              </h1>
              <p className="text-sm text-gray-600 dark:text-gray-400 mt-1">
                {stats.completed} completed, {stats.pending} pending
              </p>
            </div>
            <div className="flex items-center gap-3">
              <GlassButton
                variant="premium"
                onClick={() => setIsCreateModalOpen(true)}
                leftIcon={<Plus className="h-4 w-4" />}
                rightIcon={<Sparkles className="h-3 w-3" />}
              >
                New Task
              </GlassButton>
            </div>
          </motion.div>

          {/* Stats cards */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.3, delay: 0.1 }}
            className="grid grid-cols-3 gap-4"
          >
            {[
              { label: 'Total', value: stats.total, gradient: 'from-purple-500 to-indigo-500' },
              { label: 'Completed', value: stats.completed, gradient: 'from-emerald-500 to-green-500' },
              { label: 'Pending', value: stats.pending, gradient: 'from-amber-500 to-orange-500' },
            ].map((stat, index) => (
              <motion.div
                key={stat.label}
                initial={{ opacity: 0, scale: 0.9 }}
                animate={{ opacity: 1, scale: 1 }}
                transition={{ delay: 0.1 + index * 0.05 }}
                className={cn(
                  'p-4 rounded-2xl',
                  'bg-white/70 dark:bg-gray-900/70',
                  'backdrop-blur-xl',
                  'border border-white/30 dark:border-white/10',
                  'shadow-glass dark:shadow-glass-dark'
                )}
              >
                <div className="flex items-center gap-3">
                  <div
                    className={cn(
                      'w-2 h-8 rounded-full',
                      `bg-gradient-to-b ${stat.gradient}`
                    )}
                  />
                  <div>
                    <p className="text-2xl font-bold text-gray-900 dark:text-gray-100">{stat.value}</p>
                    <p className="text-xs text-gray-600 dark:text-gray-400">{stat.label}</p>
                  </div>
                </div>
              </motion.div>
            ))}
          </motion.div>

          {/* Filters */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.3, delay: 0.2 }}
          >
            <TaskFilters
              status={filters.status}
              priority={filters.priority}
              search={filters.search}
              onStatusChange={setStatus}
              onPriorityChange={setPriority}
              onSearchChange={setSearch}
              onClear={clearFilters}
              resultCount={resultCount}
              hasActiveFilters={hasActiveFilters}
            />
          </motion.div>

          {/* Task List */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.3, delay: 0.3 }}
          >
            <TaskList
              tasks={filteredTasks}
              isLoading={isLoading || isFetching}
              onEdit={handleEdit}
              onDelete={handleDelete}
              onToggleStatus={handleToggleStatus}
              onSetPriority={handleSetPriority}
              onSetTags={handleSetTags}
              onCreateClick={() => setIsCreateModalOpen(true)}
              onClearFilters={clearFilters}
              hasFilters={hasActiveFilters}
            />
          </motion.div>

          {/* Modals */}
          <TaskFormModal
            open={isCreateModalOpen}
            onClose={handleCloseCreateModal}
            onSuccess={handleCreateSuccess}
          />

          <TaskFormModal
            open={isEditModalOpen}
            onClose={handleCloseEditModal}
            task={editModalTask ?? undefined}
            onSuccess={handleEditSuccess}
          />

          <DeleteTaskDialog
            open={isDeleteDialogOpen}
            onClose={handleCloseDeleteDialog}
            task={deleteDialogTask}
            onSuccess={handleDeleteSuccess}
          />
        </div>
      </div>
    </div>
  );
}
