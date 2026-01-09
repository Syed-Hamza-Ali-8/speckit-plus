import { useState, useCallback, useMemo } from 'react';
import { Plus, Pencil, Trash2 } from 'lucide-react';
import { toast } from 'sonner';
import { GlassButton } from '@/components/ui/GlassButton';
import { cn } from '@/lib/utils';
import { RecurringTaskPatternModal } from '@/components/tasks/RecurringTaskPatternModal';
import { DeleteRecurringTaskPatternDialog } from '@/components/tasks/DeleteRecurringTaskPatternDialog';
import { useGetRecurringTaskPatternsQuery } from '@/services/taskApi';
import type { RecurringTaskPattern } from '@/types/task';

export function RecurringTaskPatternsPage() {
  const { data: patterns, isLoading, refetch } = useGetRecurringTaskPatternsQuery();

  const [isCreateModalOpen, setIsCreateModalOpen] = useState(false);
  const [editModalPattern, setEditModalPattern] = useState<RecurringTaskPattern | null>(null);
  const isEditModalOpen = editModalPattern !== null;
  const [deleteDialogPattern, setDeleteDialogPattern] = useState<RecurringTaskPattern | null>(null);
  const isDeleteDialogOpen = deleteDialogPattern !== null;

  const handleEdit = useCallback((pattern: RecurringTaskPattern) => {
    setEditModalPattern(pattern);
  }, []);

  const handleDelete = useCallback((pattern: RecurringTaskPattern) => {
    setDeleteDialogPattern(pattern);
  }, []);

  const handleCloseCreateModal = useCallback(() => {
    setIsCreateModalOpen(false);
  }, []);

  const handleCloseEditModal = useCallback(() => {
    setEditModalPattern(null);
  }, []);

  const handleCloseDeleteDialog = useCallback(() => {
    setDeleteDialogPattern(null);
  }, []);

  const handleCreateSuccess = useCallback(() => {
    refetch();
  }, [refetch]);

  const handleEditSuccess = useCallback(() => {
    refetch();
  }, [refetch]);

  const handleDeleteSuccess = useCallback(() => {
    refetch();
  }, [refetch]);

  const formatPatternType = (type: string) => {
    return type.charAt(0).toUpperCase() + type.slice(1);
  };

  const formatWeekdays = (weekdays: number[] | undefined) => {
    if (!weekdays || weekdays.length === 0) return 'All days';
    const dayNames = ['Sun', 'Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat'];
    return weekdays.map(day => dayNames[day]).join(', ');
  };

  const formatDaysOfMonth = (days: number[] | undefined) => {
    if (!days || days.length === 0) return 'All days';
    return days.join(', ');
  };

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
        <div className="max-w-4xl mx-auto space-y-6">
          {/* Page header */}
          <div className="flex flex-col gap-4 sm:flex-row sm:items-center sm:justify-between">
            <div>
              <h1 className="text-3xl font-bold">
                <span className="text-gradient">Recurring Task Patterns</span>
              </h1>
              <p className="text-sm text-gray-600 dark:text-gray-400 mt-1">
                Manage your recurring task patterns
              </p>
            </div>
            <GlassButton
              variant="premium"
              onClick={() => setIsCreateModalOpen(true)}
              leftIcon={<Plus className="h-4 w-4" />}
            >
              New Pattern
            </GlassButton>
          </div>

          {/* Patterns list */}
          <div className="space-y-4">
            {isLoading ? (
              <div className="text-center py-8">
                <p className="text-gray-600 dark:text-gray-400">Loading patterns...</p>
              </div>
            ) : patterns && patterns.length > 0 ? (
              patterns.map((pattern) => (
                <div
                  key={pattern.id}
                  className={cn(
                    'p-4 rounded-2xl',
                    'bg-white/70 dark:bg-gray-900/70',
                    'backdrop-blur-xl',
                    'border border-white/30 dark:border-white/10',
                    'shadow-glass dark:shadow-glass-dark',
                    'text-gray-900 dark:text-gray-100'
                  )}
                >
                  <div className="flex items-start justify-between">
                    <div className="flex-1">
                      <h3 className="font-semibold text-lg text-gray-900 dark:text-gray-100">
                        {pattern.base_task_title}
                      </h3>
                      <p className="text-sm text-gray-600 dark:text-gray-400 mt-1">
                        {pattern.base_task_description}
                      </p>
                      <div className="mt-3 grid grid-cols-2 gap-4 text-sm">
                        <div>
                          <span className="font-medium">Pattern:</span>{' '}
                          {formatPatternType(pattern.pattern_type)}
                        </div>
                        <div>
                          <span className="font-medium">Interval:</span> Every {pattern.interval}{' '}
                          {pattern.pattern_type}
                          {pattern.interval > 1 ? 's' : ''}
                        </div>
                        <div>
                          <span className="font-medium">Start Date:</span>{' '}
                          {new Date(pattern.start_date).toLocaleDateString()}
                        </div>
                        <div>
                          <span className="font-medium">End Date:</span>{' '}
                          {pattern.end_date
                            ? new Date(pattern.end_date).toLocaleDateString()
                            : 'No end date'}
                        </div>
                        {pattern.pattern_type === 'weekly' && (
                          <div>
                            <span className="font-medium">Days:</span>{' '}
                            {formatWeekdays(pattern.weekdays)}
                          </div>
                        )}
                        {pattern.pattern_type === 'monthly' && (
                          <div>
                            <span className="font-medium">Days:</span>{' '}
                            {formatDaysOfMonth(pattern.days_of_month)}
                          </div>
                        )}
                      </div>
                    </div>
                    <div className="flex items-center gap-2 ml-4">
                      <button
                        onClick={() => handleEdit(pattern)}
                        className="p-2 rounded-lg bg-white/50 dark:bg-gray-800/50 hover:bg-purple-500/10 dark:hover:bg-purple-500/20 text-gray-600 dark:text-gray-400 hover:text-purple-600 dark:hover:text-purple-400 transition-colors"
                        aria-label={`Edit pattern "${pattern.base_task_title}"`}
                      >
                        <Pencil className="h-4 w-4" />
                      </button>
                      <button
                        onClick={() => handleDelete(pattern)}
                        className="p-2 rounded-lg bg-white/50 dark:bg-gray-800/50 hover:bg-red-500/10 dark:hover:bg-red-500/20 text-gray-600 dark:text-gray-400 hover:text-red-600 dark:hover:text-red-400 transition-colors"
                        aria-label={`Delete pattern "${pattern.base_task_title}"`}
                      >
                        <Trash2 className="h-4 w-4" />
                      </button>
                    </div>
                  </div>
                </div>
              ))
            ) : (
              <div className="text-center py-8">
                <p className="text-gray-600 dark:text-gray-400">No recurring task patterns found</p>
              </div>
            )}
          </div>

          {/* Modals */}
          <RecurringTaskPatternModal
            open={isCreateModalOpen}
            onClose={handleCloseCreateModal}
            onSuccess={handleCreateSuccess}
          />

          <RecurringTaskPatternModal
            open={isEditModalOpen}
            onClose={handleCloseEditModal}
            pattern={editModalPattern ?? undefined}
            onSuccess={handleEditSuccess}
          />

          <DeleteRecurringTaskPatternDialog
            open={isDeleteDialogOpen}
            onClose={handleCloseDeleteDialog}
            pattern={deleteDialogPattern}
            onSuccess={handleDeleteSuccess}
          />
        </div>
      </div>
    </div>
  );
}