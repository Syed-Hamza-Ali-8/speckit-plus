import type { Task } from '@/types/task';
import { TaskCard } from './TaskCard';
import { TaskTable } from './TaskTable';
import { EmptyState } from './EmptyState';
import { Skeleton } from '@/components/ui/skeleton';
import { Card, CardContent } from '@/components/ui/card';

/**
 * Props for the TaskList component
 */
export interface TaskListProps {
  tasks: Task[];
  isLoading: boolean;
  onEdit: (task: Task) => void;
  onDelete: (task: Task) => void;
  onToggleStatus: (task: Task) => void;
  onCreateClick: () => void;
  onClearFilters: () => void;
  hasFilters: boolean;
}

/**
 * Loading skeleton for card layout
 */
function CardSkeleton() {
  return (
    <Card>
      <CardContent className="p-4">
        <div className="flex items-start gap-3">
          <Skeleton className="h-4 w-4 mt-1 rounded" />
          <div className="flex-1 space-y-2">
            <Skeleton className="h-4 w-3/4" />
            <Skeleton className="h-3 w-1/2" />
            <div className="flex gap-2 mt-2">
              <Skeleton className="h-5 w-16 rounded-full" />
              <Skeleton className="h-4 w-20" />
            </div>
          </div>
          <div className="flex gap-1">
            <Skeleton className="h-8 w-8 rounded" />
            <Skeleton className="h-8 w-8 rounded" />
          </div>
        </div>
      </CardContent>
    </Card>
  );
}

/**
 * Loading skeleton for table layout
 */
function TableSkeleton() {
  return (
    <div className="border rounded-lg">
      {/* Header skeleton */}
      <div className="border-b p-4 flex gap-4 bg-muted/50">
        <Skeleton className="h-4 w-8" />
        <Skeleton className="h-4 w-32" />
        <Skeleton className="h-4 w-48 hidden md:block" />
        <Skeleton className="h-4 w-20" />
        <Skeleton className="h-4 w-24 hidden sm:block" />
        <Skeleton className="h-4 w-8" />
      </div>
      {/* Row skeletons */}
      {[1, 2, 3, 4, 5].map((i) => (
        <div key={i} className="border-b last:border-0 p-4 flex gap-4 items-center">
          <Skeleton className="h-4 w-4 rounded" />
          <Skeleton className="h-4 w-32" />
          <Skeleton className="h-4 w-48 hidden md:block" />
          <Skeleton className="h-5 w-20 rounded-full" />
          <Skeleton className="h-4 w-24 hidden sm:block" />
          <Skeleton className="h-8 w-8 rounded" />
        </div>
      ))}
    </div>
  );
}

/**
 * TaskList Component
 *
 * Responsive wrapper that displays tasks differently based on screen size:
 * - Mobile (<640px): Stacked TaskCard components
 * - Tablet (640-1024px): 2-column grid of TaskCard components
 * - Desktop (>1024px): TaskTable component
 *
 * Also handles loading and empty states.
 */
export function TaskList({
  tasks,
  isLoading,
  onEdit,
  onDelete,
  onToggleStatus,
  onCreateClick,
  onClearFilters,
  hasFilters,
}: TaskListProps) {
  // T051: Loading skeleton state
  if (isLoading) {
    return (
      <>
        {/* Mobile/Tablet: Card skeletons */}
        <div className="lg:hidden space-y-3 sm:grid sm:grid-cols-2 sm:gap-4 sm:space-y-0">
          {[1, 2, 3, 4].map((i) => (
            <CardSkeleton key={i} />
          ))}
        </div>
        {/* Desktop: Table skeleton */}
        <div className="hidden lg:block">
          <TableSkeleton />
        </div>
      </>
    );
  }

  // T052: Empty state when no tasks
  if (tasks.length === 0) {
    return (
      <EmptyState
        hasFilters={hasFilters}
        onCreateClick={onCreateClick}
        onClearFilters={onClearFilters}
      />
    );
  }

  return (
    <>
      {/* T048 & T049: Mobile (<640px) and Tablet (640-1024px) card layouts */}
      <div className="lg:hidden">
        {/* Mobile: single column stack */}
        <div className="sm:hidden space-y-3">
          {tasks.map((task) => (
            <TaskCard
              key={task.id}
              task={task}
              onEdit={() => onEdit(task)}
              onDelete={() => onDelete(task)}
              onToggleStatus={() => onToggleStatus(task)}
            />
          ))}
        </div>
        {/* Tablet: 2-column grid */}
        <div className="hidden sm:grid sm:grid-cols-2 gap-4">
          {tasks.map((task) => (
            <TaskCard
              key={task.id}
              task={task}
              onEdit={() => onEdit(task)}
              onDelete={() => onDelete(task)}
              onToggleStatus={() => onToggleStatus(task)}
            />
          ))}
        </div>
      </div>

      {/* T050: Desktop (>1024px) table layout */}
      <div className="hidden lg:block">
        <TaskTable
          tasks={tasks}
          onEdit={onEdit}
          onDelete={onDelete}
          onToggleStatus={onToggleStatus}
        />
      </div>
    </>
  );
}

export default TaskList;
