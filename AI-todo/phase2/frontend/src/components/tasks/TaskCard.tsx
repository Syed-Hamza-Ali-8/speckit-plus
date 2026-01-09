import { motion } from 'framer-motion';
import { Pencil, Trash2, Calendar, AlertTriangle } from 'lucide-react';
import { GlassCard } from '@/components/ui/GlassCard';
import { StatusBadge } from '@/components/ui/StatusBadge';
import { Checkbox } from '@/components/ui/checkbox';
import { cn } from '@/lib/utils';
import type { Task } from '@/types/task';

export interface TaskCardProps {
  task: Task;
  onEdit: () => void;
  onDelete: () => void;
  onToggleStatus: () => void;
  onSetPriority: (priority: 'low' | 'medium' | 'high') => void;
  onSetTags: (tags: string[]) => void;
  index?: number;
}

function truncateText(text: string, maxLength: number): string {
  if (text.length <= maxLength) return text;
  return text.slice(0, maxLength).trim() + '...';
}

function formatDate(isoDate: string): string {
  const date = new Date(isoDate);
  return date.toLocaleDateString('en-US', {
    month: 'short',
    day: 'numeric',
    year: 'numeric',
  });
}

function formatDueDate(isoDate: string): string {
  const date = new Date(isoDate + 'T00:00:00');
  return date.toLocaleDateString('en-US', {
    month: 'short',
    day: 'numeric',
  });
}

function isOverdue(dueDate: string | null, status: string): boolean {
  if (!dueDate || status === 'completed') return false;
  const today = new Date();
  today.setHours(0, 0, 0, 0);
  const due = new Date(dueDate + 'T00:00:00');
  return due < today;
}

function isDueSoon(dueDate: string | null, status: string): boolean {
  if (!dueDate || status === 'completed') return false;
  const today = new Date();
  today.setHours(0, 0, 0, 0);
  const due = new Date(dueDate + 'T00:00:00');
  const diffDays = Math.ceil((due.getTime() - today.getTime()) / (1000 * 60 * 60 * 24));
  return diffDays >= 0 && diffDays <= 2;
}

export function TaskCard({
  task,
  onEdit,
  onDelete,
  onToggleStatus,
  index = 0,
}: TaskCardProps) {
  const isCompleted = task.status === 'completed';
  const taskIsOverdue = isOverdue(task.due_date, task.status);
  const taskIsDueSoon = isDueSoon(task.due_date, task.status);

  return (
    <motion.div
      initial={{ opacity: 0, y: 20 }}
      animate={{ opacity: 1, y: 0 }}
      exit={{ opacity: 0, y: -20, scale: 0.95 }}
      transition={{
        duration: 0.3,
        delay: index * 0.05,
        ease: [0.4, 0, 0.2, 1],
      }}
      layout
      layoutId={task.id}
    >
      <GlassCard
        variant="default"
        hover={true}
        padding="none"
        className={cn(
          'group relative overflow-hidden',
          isCompleted && 'opacity-75'
        )}
      >
        {/* Gradient accent bar */}
        <div
          className={cn(
            'absolute left-0 top-0 bottom-0 w-1',
            'bg-gradient-to-b',
            isCompleted
              ? 'from-emerald-500 to-green-500'
              : taskIsOverdue
              ? 'from-red-500 to-red-600'
              : taskIsDueSoon
              ? 'from-amber-500 to-orange-500'
              : 'from-purple-500 via-blue-500 to-indigo-600'
          )}
        />

        <div className="p-4 pl-5">
          <div className="flex items-start gap-3">
            {/* Checkbox */}
            <motion.div
              whileHover={{ scale: 1.1 }}
              whileTap={{ scale: 0.9 }}
            >
              <Checkbox
                checked={isCompleted}
                onCheckedChange={onToggleStatus}
                className={cn(
                  'mt-1 h-5 w-5 rounded-md',
                  'border-2',
                  isCompleted
                    ? 'border-emerald-500 bg-emerald-500 data-[state=checked]:bg-emerald-500'
                    : 'border-purple-300 dark:border-purple-600'
                )}
                aria-label={`Mark task "${task.title}" as ${isCompleted ? 'pending' : 'completed'}`}
              />
            </motion.div>

            <div className="flex-1 min-w-0">
              {/* Title */}
              <h3
                className={cn(
                  'font-medium text-sm leading-tight transition-all duration-200',
                  isCompleted
                    ? 'line-through text-gray-500 dark:text-gray-400'
                    : 'text-gray-900 dark:text-gray-100'
                )}
                title={task.title}
              >
                {truncateText(task.title, 50)}
              </h3>

              {/* Description */}
              {task.description && (
                <p
                  className="text-xs text-gray-600 dark:text-gray-400 mt-1.5 line-clamp-2"
                  title={task.description}
                >
                  {truncateText(task.description, 100)}
                </p>
              )}

              {/* Status badge, date, and advanced features */}
              <div className="flex items-center gap-2 mt-3 flex-wrap">
                <StatusBadge
                  status={isCompleted ? 'completed' : 'pending'}
                  size="sm"
                />
                {task.due_date && (
                  <div
                    className={cn(
                      'flex items-center gap-1 px-2 py-0.5 rounded-md text-xs font-medium',
                      taskIsOverdue
                        ? 'bg-red-100 dark:bg-red-900/30 text-red-600 dark:text-red-400'
                        : taskIsDueSoon
                        ? 'bg-amber-100 dark:bg-amber-900/30 text-amber-600 dark:text-amber-400'
                        : 'bg-gray-100 dark:bg-gray-800 text-gray-600 dark:text-gray-400'
                    )}
                  >
                    {taskIsOverdue ? (
                      <AlertTriangle className="w-3 h-3" />
                    ) : (
                      <Calendar className="w-3 h-3" />
                    )}
                    <span>{taskIsOverdue ? 'Overdue' : formatDueDate(task.due_date)}</span>
                  </div>
                )}
                {/* Priority indicator */}
                {task.priority && (
                  <div
                    className={cn(
                      'px-2 py-0.5 rounded-md text-xs font-medium',
                      task.priority === 'high'
                        ? 'bg-red-100 dark:bg-red-900/30 text-red-600 dark:text-red-400'
                        : task.priority === 'medium'
                        ? 'bg-amber-100 dark:bg-amber-900/30 text-amber-600 dark:text-amber-400'
                        : 'bg-green-100 dark:bg-green-900/30 text-green-600 dark:text-green-400'
                    )}
                  >
                    {task.priority.charAt(0).toUpperCase() + task.priority.slice(1)}
                  </div>
                )}
                {/* Recurring indicator */}
                {task.is_recurring && (
                  <div className="px-2 py-0.5 rounded-md text-xs font-medium bg-blue-100 dark:bg-blue-900/30 text-blue-600 dark:text-blue-400">
                    Recurring
                  </div>
                )}
                <span className="text-xs text-gray-600 dark:text-gray-400">
                  {formatDate(task.created_at)}
                </span>
              </div>

              {/* Tags display */}
              {task.tags && task.tags.length > 0 && (
                <div className="flex flex-wrap gap-1 mt-2">
                  {task.tags.map((tag, index) => (
                    <span
                      key={index}
                      className="px-2 py-0.5 rounded-md text-xs bg-purple-100 dark:bg-purple-900/30 text-purple-600 dark:text-purple-400"
                    >
                      {tag}
                    </span>
                  ))}
                  <button
                    onClick={onEdit}
                    className="px-2 py-0.5 rounded-md text-xs bg-gray-100 dark:bg-gray-800 text-gray-600 dark:text-gray-400 hover:bg-gray-200 dark:hover:bg-gray-700 transition-colors"
                  >
                    Edit Tags
                  </button>
                </div>
              )}
              {!task.tags || task.tags.length === 0 && (
                <button
                  onClick={onEdit}
                  className="mt-2 px-3 py-1 rounded-md text-xs bg-purple-100 dark:bg-purple-900/30 text-purple-600 dark:text-purple-400 hover:bg-purple-200 dark:hover:bg-purple-800/50 transition-colors"
                >
                  Add Tags
                </button>
              )}
            </div>

            {/* Actions */}
            <div className="flex items-center gap-1 opacity-0 group-hover:opacity-100 transition-opacity duration-200">
              {/* Priority buttons */}
              <div className="flex gap-1">
                {(['low', 'medium', 'high'] as const).map((priority) => (
                  <motion.button
                    key={priority}
                    whileHover={{ scale: 1.1 }}
                    whileTap={{ scale: 0.9 }}
                    onClick={() => onSetPriority(priority)}
                    className={cn(
                      'p-1.5 rounded',
                      priority === 'low'
                        ? task.priority === 'low'
                          ? 'bg-green-500 text-white'
                          : 'bg-green-100 dark:bg-green-900/30 text-green-600 dark:text-green-400'
                        : priority === 'medium'
                        ? task.priority === 'medium'
                          ? 'bg-amber-500 text-white'
                          : 'bg-amber-100 dark:bg-amber-900/30 text-amber-600 dark:text-amber-400'
                        : task.priority === 'high'
                        ? 'bg-red-500 text-white'
                        : 'bg-red-100 dark:bg-red-900/30 text-red-600 dark:text-red-400',
                      'transition-colors duration-200'
                    )}
                    aria-label={`Set task priority to ${priority}`}
                  >
                    <span className="text-xs font-bold">
                      {priority.charAt(0).toUpperCase()}
                    </span>
                  </motion.button>
                ))}
              </div>
              <motion.button
                whileHover={{ scale: 1.1 }}
                whileTap={{ scale: 0.9 }}
                onClick={onEdit}
                className={cn(
                  'p-2 rounded-lg',
                  'bg-white/50 dark:bg-gray-800/50',
                  'hover:bg-purple-500/10 dark:hover:bg-purple-500/20',
                  'text-muted-foreground hover:text-purple-600 dark:hover:text-purple-400',
                  'transition-colors duration-200'
                )}
                aria-label={`Edit task "${task.title}"`}
              >
                <Pencil className="h-4 w-4" />
              </motion.button>
              <motion.button
                whileHover={{ scale: 1.1 }}
                whileTap={{ scale: 0.9 }}
                onClick={onDelete}
                className={cn(
                  'p-2 rounded-lg',
                  'bg-white/50 dark:bg-gray-800/50',
                  'hover:bg-red-500/10 dark:hover:bg-red-500/20',
                  'text-muted-foreground hover:text-red-600 dark:hover:text-red-400',
                  'transition-colors duration-200'
                )}
                aria-label={`Delete task "${task.title}"`}
              >
                <Trash2 className="h-4 w-4" />
              </motion.button>
            </div>
          </div>
        </div>
      </GlassCard>
    </motion.div>
  );
}

export default TaskCard;
