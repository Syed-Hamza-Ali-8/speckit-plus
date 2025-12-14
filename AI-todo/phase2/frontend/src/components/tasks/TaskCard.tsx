import { Pencil, Trash2 } from 'lucide-react';
import { Card, CardContent } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';
import { Checkbox } from '@/components/ui/checkbox';
import { Button } from '@/components/ui/button';
import type { Task } from '@/types/task';

/**
 * Props for the TaskCard component
 */
export interface TaskCardProps {
  task: Task;
  onEdit: () => void;
  onDelete: () => void;
  onToggleStatus: () => void;
}

/**
 * Truncate text with ellipsis if exceeds max length
 */
function truncateText(text: string, maxLength: number): string {
  if (text.length <= maxLength) return text;
  return text.slice(0, maxLength).trim() + '...';
}

/**
 * Format ISO date string to readable format (e.g., "Dec 15, 2025")
 */
function formatDate(isoDate: string): string {
  const date = new Date(isoDate);
  return date.toLocaleDateString('en-US', {
    month: 'short',
    day: 'numeric',
    year: 'numeric',
  });
}

/**
 * TaskCard Component
 *
 * Mobile-friendly card display for a single task.
 * Shows title, description, status badge, created date, and action buttons.
 */
export function TaskCard({ task, onEdit, onDelete, onToggleStatus }: TaskCardProps) {
  const isCompleted = task.status === 'completed';

  return (
    <Card className="w-full transition-colors hover:bg-accent/50">
      <CardContent className="p-4">
        <div className="flex items-start gap-3">
          {/* T017: Checkbox for status toggle */}
          <Checkbox
            checked={isCompleted}
            onCheckedChange={onToggleStatus}
            className="mt-1"
            aria-label={`Mark task "${task.title}" as ${isCompleted ? 'pending' : 'completed'}`}
          />

          <div className="flex-1 min-w-0">
            {/* T013: Task title (truncated if > 50 chars) */}
            <h3
              className={`font-medium text-sm leading-tight ${
                isCompleted ? 'line-through text-muted-foreground' : ''
              }`}
              title={task.title}
            >
              {truncateText(task.title, 50)}
            </h3>

            {/* T014: Task description (truncated, optional) */}
            {task.description && (
              <p
                className="text-xs text-muted-foreground mt-1 line-clamp-2"
                title={task.description}
              >
                {truncateText(task.description, 100)}
              </p>
            )}

            {/* T015 & T016: Status badge and created date */}
            <div className="flex items-center gap-2 mt-2">
              <Badge
                variant={isCompleted ? 'default' : 'secondary'}
                className={
                  isCompleted
                    ? 'bg-green-500/10 text-green-600 dark:text-green-400 border-green-500/20'
                    : 'bg-yellow-500/10 text-yellow-600 dark:text-yellow-400 border-yellow-500/20'
                }
              >
                {isCompleted ? 'Completed' : 'Pending'}
              </Badge>
              <span className="text-xs text-muted-foreground">
                {formatDate(task.created_at)}
              </span>
            </div>
          </div>

          {/* T018 & T019: Action buttons */}
          <div className="flex items-center gap-1">
            <Button
              variant="ghost"
              size="icon"
              className="h-8 w-8"
              onClick={onEdit}
              aria-label={`Edit task "${task.title}"`}
            >
              <Pencil className="h-4 w-4" />
            </Button>
            <Button
              variant="ghost"
              size="icon"
              className="h-8 w-8 text-destructive hover:text-destructive hover:bg-destructive/10"
              onClick={onDelete}
              aria-label={`Delete task "${task.title}"`}
            >
              <Trash2 className="h-4 w-4" />
            </Button>
          </div>
        </div>
      </CardContent>
    </Card>
  );
}

export default TaskCard;
