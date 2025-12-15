import { motion } from 'framer-motion';
import { MoreHorizontal, Pencil, Trash2 } from 'lucide-react';
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from '@/components/ui/table';
import { StatusBadge } from '@/components/ui/StatusBadge';
import { Checkbox } from '@/components/ui/checkbox';
import { Button } from '@/components/ui/button';
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
} from '@/components/ui/dropdown-menu';
import { cn } from '@/lib/utils';
import type { Task } from '@/types/task';

export interface TaskTableProps {
  tasks: Task[];
  onEdit: (task: Task) => void;
  onDelete: (task: Task) => void;
  onToggleStatus: (task: Task) => void;
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

export function TaskTable({ tasks, onEdit, onDelete, onToggleStatus }: TaskTableProps) {
  return (
    <div
      className={cn(
        'rounded-2xl overflow-hidden',
        'bg-white/70 dark:bg-gray-900/70',
        'backdrop-blur-xl',
        'border border-white/30 dark:border-white/10',
        'shadow-glass dark:shadow-glass-dark',
        'text-gray-900 dark:text-gray-100'
      )}
    >
      <Table>
        <TableHeader>
          <TableRow className="border-b border-white/20 dark:border-white/5 hover:bg-transparent">
            <TableHead className="w-12 bg-white/50 dark:bg-gray-900/50"></TableHead>
            <TableHead className="bg-white/50 dark:bg-gray-900/50 font-semibold text-gray-700 dark:text-gray-300">Title</TableHead>
            <TableHead className="hidden md:table-cell bg-white/50 dark:bg-gray-900/50 font-semibold text-gray-700 dark:text-gray-300">
              Description
            </TableHead>
            <TableHead className="bg-white/50 dark:bg-gray-900/50 font-semibold text-gray-700 dark:text-gray-300">Status</TableHead>
            <TableHead className="hidden sm:table-cell bg-white/50 dark:bg-gray-900/50 font-semibold text-gray-700 dark:text-gray-300">
              Created
            </TableHead>
            <TableHead className="w-12 bg-white/50 dark:bg-gray-900/50"></TableHead>
          </TableRow>
        </TableHeader>
        <TableBody>
          {tasks.map((task, index) => {
            const isCompleted = task.status === 'completed';
            return (
              <motion.tr
                key={task.id}
                initial={{ opacity: 0, x: -20 }}
                animate={{ opacity: 1, x: 0 }}
                transition={{ delay: index * 0.03, duration: 0.2 }}
                className={cn(
                  'group border-b border-white/10 dark:border-white/5 last:border-0',
                  'hover:bg-purple-500/5 dark:hover:bg-purple-500/10',
                  'transition-colors duration-200',
                  isCompleted && 'opacity-60'
                )}
              >
                {/* Checkbox column */}
                <TableCell className="py-3">
                  <motion.div whileHover={{ scale: 1.1 }} whileTap={{ scale: 0.9 }}>
                    <Checkbox
                      checked={isCompleted}
                      onCheckedChange={() => onToggleStatus(task)}
                      className={cn(
                        'h-5 w-5 rounded-md border-2',
                        isCompleted
                          ? 'border-emerald-500 bg-emerald-500 data-[state=checked]:bg-emerald-500'
                          : 'border-purple-300 dark:border-purple-600'
                      )}
                      aria-label={`Mark "${task.title}" as ${isCompleted ? 'pending' : 'completed'}`}
                    />
                  </motion.div>
                </TableCell>

                {/* Title column */}
                <TableCell className="py-3">
                  <span
                    className={cn(
                      'font-medium transition-all duration-200',
                      isCompleted ? 'line-through text-muted-foreground' : ''
                    )}
                    title={task.title}
                  >
                    {truncateText(task.title, 40)}
                  </span>
                </TableCell>

                {/* Description column */}
                <TableCell className="hidden md:table-cell text-gray-600 dark:text-gray-400 py-3">
                  {task.description ? (
                    <span title={task.description}>{truncateText(task.description, 50)}</span>
                  ) : (
                    <span className="text-gray-400 dark:text-gray-500">-</span>
                  )}
                </TableCell>

                {/* Status column */}
                <TableCell className="py-3">
                  <StatusBadge status={isCompleted ? 'completed' : 'pending'} size="sm" />
                </TableCell>

                {/* Created column */}
                <TableCell className="hidden sm:table-cell text-gray-600 dark:text-gray-400 py-3">
                  {formatDate(task.created_at)}
                </TableCell>

                {/* Actions column */}
                <TableCell className="py-3">
                  <DropdownMenu>
                    <DropdownMenuTrigger asChild>
                      <Button
                        variant="ghost"
                        size="icon"
                        className={cn(
                          'h-8 w-8 rounded-lg',
                          'opacity-0 group-hover:opacity-100',
                          'hover:bg-purple-500/10 dark:hover:bg-purple-500/20',
                          'transition-all duration-200'
                        )}
                      >
                        <MoreHorizontal className="h-4 w-4" />
                        <span className="sr-only">Open menu</span>
                      </Button>
                    </DropdownMenuTrigger>
                    <DropdownMenuContent
                      align="end"
                      className={cn(
                        'bg-white/90 dark:bg-gray-900/90',
                        'backdrop-blur-xl',
                        'border border-white/30 dark:border-white/10',
                        'shadow-xl rounded-xl'
                      )}
                    >
                      <DropdownMenuItem
                        onClick={() => onEdit(task)}
                        className="hover:bg-purple-500/10 rounded-lg cursor-pointer"
                      >
                        <Pencil className="mr-2 h-4 w-4" />
                        Edit
                      </DropdownMenuItem>
                      <DropdownMenuItem
                        onClick={() => onDelete(task)}
                        className="text-red-600 dark:text-red-400 hover:bg-red-500/10 rounded-lg cursor-pointer focus:text-red-600 dark:focus:text-red-400"
                      >
                        <Trash2 className="mr-2 h-4 w-4" />
                        Delete
                      </DropdownMenuItem>
                    </DropdownMenuContent>
                  </DropdownMenu>
                </TableCell>
              </motion.tr>
            );
          })}
        </TableBody>
      </Table>
    </div>
  );
}

export default TaskTable;
