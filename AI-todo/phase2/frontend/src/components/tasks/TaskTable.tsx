import { MoreHorizontal, Pencil, Trash2 } from 'lucide-react';
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from '@/components/ui/table';
import { Badge } from '@/components/ui/badge';
import { Checkbox } from '@/components/ui/checkbox';
import { Button } from '@/components/ui/button';
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
} from '@/components/ui/dropdown-menu';
import type { Task } from '@/types/task';

/**
 * Props for the TaskTable component
 */
export interface TaskTableProps {
  tasks: Task[];
  onEdit: (task: Task) => void;
  onDelete: (task: Task) => void;
  onToggleStatus: (task: Task) => void;
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
 * TaskTable Component
 *
 * Desktop table view for tasks with columns:
 * - Checkbox (status toggle)
 * - Title
 * - Description
 * - Status badge
 * - Created date
 * - Actions dropdown
 */
export function TaskTable({ tasks, onEdit, onDelete, onToggleStatus }: TaskTableProps) {
  return (
    <Table>
      <TableHeader>
        <TableRow>
          <TableHead className="w-12"></TableHead>
          <TableHead>Title</TableHead>
          <TableHead className="hidden md:table-cell">Description</TableHead>
          <TableHead>Status</TableHead>
          <TableHead className="hidden sm:table-cell">Created</TableHead>
          <TableHead className="w-12"></TableHead>
        </TableRow>
      </TableHeader>
      <TableBody>
        {tasks.map((task) => {
          const isCompleted = task.status === 'completed';
          return (
            <TableRow key={task.id}>
              {/* T029: Checkbox column */}
              <TableCell>
                <Checkbox
                  checked={isCompleted}
                  onCheckedChange={() => onToggleStatus(task)}
                  aria-label={`Mark "${task.title}" as ${isCompleted ? 'pending' : 'completed'}`}
                />
              </TableCell>

              {/* T030: Title column */}
              <TableCell>
                <span
                  className={`font-medium ${isCompleted ? 'line-through text-muted-foreground' : ''}`}
                  title={task.title}
                >
                  {truncateText(task.title, 40)}
                </span>
              </TableCell>

              {/* T031: Description column */}
              <TableCell className="hidden md:table-cell text-muted-foreground">
                {task.description ? (
                  <span title={task.description}>{truncateText(task.description, 50)}</span>
                ) : (
                  <span className="text-muted-foreground/50">—</span>
                )}
              </TableCell>

              {/* T032: Status column */}
              <TableCell>
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
              </TableCell>

              {/* T033: Created column */}
              <TableCell className="hidden sm:table-cell text-muted-foreground">
                {formatDate(task.created_at)}
              </TableCell>

              {/* T034: Actions column with dropdown */}
              <TableCell>
                <DropdownMenu>
                  <DropdownMenuTrigger asChild>
                    <Button variant="ghost" size="icon" className="h-8 w-8">
                      <MoreHorizontal className="h-4 w-4" />
                      <span className="sr-only">Open menu</span>
                    </Button>
                  </DropdownMenuTrigger>
                  <DropdownMenuContent align="end">
                    <DropdownMenuItem onClick={() => onEdit(task)}>
                      <Pencil className="mr-2 h-4 w-4" />
                      Edit
                    </DropdownMenuItem>
                    <DropdownMenuItem
                      onClick={() => onDelete(task)}
                      className="text-destructive focus:text-destructive"
                    >
                      <Trash2 className="mr-2 h-4 w-4" />
                      Delete
                    </DropdownMenuItem>
                  </DropdownMenuContent>
                </DropdownMenu>
              </TableCell>
            </TableRow>
          );
        })}
      </TableBody>
    </Table>
  );
}

export default TaskTable;
