import { AnimatePresence, motion } from 'framer-motion';
import type { Task } from '@/types/task';
import { TaskCard } from './TaskCard';
import { TaskTable } from './TaskTable';
import { EmptyState } from './EmptyState';
import { SkeletonTask, SkeletonTable } from '@/components/ui/ShimmerSkeleton';

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

function CardSkeletonList() {
  return (
    <div className="space-y-3 sm:grid sm:grid-cols-2 sm:gap-4 sm:space-y-0">
      {[1, 2, 3, 4].map((i) => (
        <motion.div
          key={i}
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: i * 0.1 }}
        >
          <SkeletonTask />
        </motion.div>
      ))}
    </div>
  );
}

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
  // Loading skeleton state
  if (isLoading) {
    return (
      <>
        {/* Mobile/Tablet: Card skeletons */}
        <div className="lg:hidden">
          <CardSkeletonList />
        </div>
        {/* Desktop: Table skeleton */}
        <div className="hidden lg:block">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.3 }}
          >
            <SkeletonTable rows={5} />
          </motion.div>
        </div>
      </>
    );
  }

  // Empty state when no tasks
  if (tasks.length === 0) {
    return (
      <motion.div
        initial={{ opacity: 0, scale: 0.95 }}
        animate={{ opacity: 1, scale: 1 }}
        transition={{ duration: 0.3 }}
      >
        <EmptyState
          hasFilters={hasFilters}
          onCreateClick={onCreateClick}
          onClearFilters={onClearFilters}
        />
      </motion.div>
    );
  }

  const containerVariants = {
    hidden: { opacity: 0 },
    visible: {
      opacity: 1,
      transition: {
        staggerChildren: 0.05,
      },
    },
  };

  return (
    <>
      {/* Mobile (<640px) and Tablet (640-1024px) card layouts */}
      <div className="lg:hidden">
        {/* Mobile: single column stack */}
        <motion.div
          className="sm:hidden space-y-3"
          variants={containerVariants}
          initial="hidden"
          animate="visible"
        >
          <AnimatePresence mode="popLayout">
            {tasks.map((task, index) => (
              <TaskCard
                key={task.id}
                task={task}
                index={index}
                onEdit={() => onEdit(task)}
                onDelete={() => onDelete(task)}
                onToggleStatus={() => onToggleStatus(task)}
              />
            ))}
          </AnimatePresence>
        </motion.div>

        {/* Tablet: 2-column grid */}
        <motion.div
          className="hidden sm:grid sm:grid-cols-2 gap-4"
          variants={containerVariants}
          initial="hidden"
          animate="visible"
        >
          <AnimatePresence mode="popLayout">
            {tasks.map((task, index) => (
              <TaskCard
                key={task.id}
                task={task}
                index={index}
                onEdit={() => onEdit(task)}
                onDelete={() => onDelete(task)}
                onToggleStatus={() => onToggleStatus(task)}
              />
            ))}
          </AnimatePresence>
        </motion.div>
      </div>

      {/* Desktop (>1024px) table layout */}
      <motion.div
        className="hidden lg:block"
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.3 }}
      >
        <TaskTable
          tasks={tasks}
          onEdit={onEdit}
          onDelete={onDelete}
          onToggleStatus={onToggleStatus}
        />
      </motion.div>
    </>
  );
}

export default TaskList;
