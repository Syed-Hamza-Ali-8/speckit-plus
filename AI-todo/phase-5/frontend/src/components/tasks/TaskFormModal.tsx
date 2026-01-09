import { useEffect } from 'react';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { toast } from 'sonner';
import { CalendarIcon, X } from 'lucide-react';
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogFooter,
} from '@/components/ui/dialog';
import { Input } from '@/components/ui/input';
import { Label } from '@/components/ui/label';
import { Textarea } from '@/components/ui/textarea';
import { Checkbox } from '@/components/ui/checkbox';
import { Button } from '@/components/ui/button';
import { LoadingButton } from '@/components/ui/loading-button';
import { taskSchema, taskEditSchema } from '@/lib/validations/task';
import type { TaskFormData, TaskEditFormData } from '@/lib/validations/task';
import { useCreateTaskMutation, useUpdateTaskMutation } from '@/services/taskApi';
import type { Task } from '@/types/task';
import { cn } from '@/lib/utils';

/**
 * Props for the TaskFormModal component
 */
export interface TaskFormModalProps {
  open: boolean;
  onClose: () => void;
  task?: Task; // If provided, edit mode
  onSuccess: () => void;
}

/**
 * TaskFormModal Component
 *
 * Modal dialog for creating and editing tasks.
 * Features:
 * - React Hook Form with Zod validation
 * - Create mode: title + description
 * - Edit mode: title + description + status toggle
 * - RTK Query mutations for API calls
 * - Success/error toast notifications
 * - Form reset on close
 */
export function TaskFormModal({ open, onClose, task, onSuccess }: TaskFormModalProps) {
  const isEditMode = !!task;

  // T060 & T061: RTK Query mutations
  const [createTask, { isLoading: isCreating }] = useCreateTaskMutation();
  const [updateTask, { isLoading: isUpdating }] = useUpdateTaskMutation();

  const isLoading = isCreating || isUpdating;

  // T054: React Hook Form setup with Zod resolver
  const {
    register,
    handleSubmit,
    reset,
    setValue,
    watch,
    formState: { errors },
  } = useForm<TaskEditFormData>({
    resolver: zodResolver(isEditMode ? taskEditSchema : taskSchema),
    defaultValues: {
      title: '',
      description: '',
      status: 'pending',
      dueDate: '',
      priority: 'medium',
      tags: [],
    },
  });

  // T059: Pre-populate form with task data in edit mode
  useEffect(() => {
    if (task && open) {
      reset({
        title: task.title,
        description: task.description || '',
        status: task.status,
        dueDate: task.due_date || '',
        priority: task.priority,
        tags: task.tags || [],
      });
    } else if (!open) {
      // T066: Reset form on modal close
      reset({
        title: '',
        description: '',
        status: 'pending',
        dueDate: '',
        priority: 'medium',
        tags: [],
      });
    }
  }, [task, open, reset]);

  // Watch status for checkbox
  const currentStatus = watch('status');
  const currentDueDate = watch('dueDate');

  // Get today's date in YYYY-MM-DD format for min attribute
  const today = new Date().toISOString().split('T')[0];

  // Form submission handler
  const onSubmit = async (data: TaskFormData | TaskEditFormData) => {
    try {
      if (isEditMode && task) {
        // T061: Update existing task - only send changed fields
        const updateData: {
          title?: string;
          description?: string;
          status?: 'pending' | 'completed';
          due_date?: string | null;
          priority?: 'low' | 'medium' | 'high';
          tags?: string[];
        } = {};

        // Only include title if changed
        if (data.title !== task.title) {
          updateData.title = data.title;
        }

        // Only include description if changed
        const newDescription = data.description || null;
        const oldDescription = task.description || null;
        if (newDescription !== oldDescription) {
          updateData.description = data.description || undefined;
        }

        // Only include status if changed
        const formStatus = (data as TaskEditFormData).status;
        if (formStatus && formStatus !== task.status) {
          updateData.status = formStatus;
        }

        // Only include due_date if changed
        const newDueDate = data.dueDate || null;
        const oldDueDate = task.due_date || null;
        if (newDueDate !== oldDueDate) {
          updateData.due_date = newDueDate;
        }

        // Only include priority if changed
        const formPriority = (data as TaskEditFormData).priority;
        if (formPriority && formPriority !== task.priority) {
          updateData.priority = formPriority;
        }

        // Only include tags if changed
        const formTags = (data as TaskEditFormData).tags || [];
        const oldTags = task.tags || [];
        if (JSON.stringify(formTags) !== JSON.stringify(oldTags)) {
          updateData.tags = formTags;
        }

        // Only make API call if something changed
        if (Object.keys(updateData).length === 0) {
          toast.info('No changes to save');
          onClose();
          return;
        }

        await updateTask({
          id: task.id,
          data: updateData,
        }).unwrap();
        // T063: Success toast for edit
        toast.success('Task updated!');
      } else {
        // T060: Create new task
        await createTask({
          title: data.title,
          description: data.description || undefined,
          due_date: data.dueDate || undefined,
          priority: data.priority,
          tags: data.tags,
          is_recurring: data.isRecurring,
        }).unwrap();
        // T063: Success toast for create
        toast.success('Task created!');
      }
      onSuccess();
      onClose();
    } catch (error) {
      // T064: Error toast with API error message
      const errorMessage = error instanceof Error ? error.message : 'An error occurred';
      toast.error(errorMessage);
    }
  };

  // T065: Handle modal close (Cancel button, Escape key, outside click handled by Dialog)
  const handleClose = () => {
    if (!isLoading) {
      onClose();
    }
  };

  return (
    <Dialog open={open} onOpenChange={(isOpen) => !isOpen && handleClose()}>
      <DialogContent className="sm:max-w-[425px]">
        <DialogHeader>
          {/* T058: Conditional title */}
          <DialogTitle>{isEditMode ? 'Edit Task' : 'New Task'}</DialogTitle>
        </DialogHeader>

        <form onSubmit={handleSubmit(onSubmit)} className="space-y-4">
          {/* T055: Title Input field */}
          <div className="space-y-2">
            <Label htmlFor="title">Title</Label>
            <Input
              id="title"
              placeholder="Enter task title..."
              {...register('title')}
              aria-invalid={!!errors.title}
            />
            {errors.title && (
              <p className="text-sm text-destructive">{errors.title.message}</p>
            )}
          </div>

          {/* T056: Description Textarea field */}
          <div className="space-y-2">
            <Label htmlFor="description">Description (optional)</Label>
            <Textarea
              id="description"
              placeholder="Enter task description..."
              rows={3}
              {...register('description')}
              aria-invalid={!!errors.description}
            />
            {errors.description && (
              <p className="text-sm text-destructive">{errors.description.message}</p>
            )}
          </div>

          {/* Due Date field */}
          <div className="space-y-2">
            <Label htmlFor="dueDate">Due Date (optional)</Label>
            <div className="relative">
              <CalendarIcon className="absolute left-3 top-1/2 -translate-y-1/2 h-4 w-4 text-gray-500 dark:text-gray-400" />
              <Input
                id="dueDate"
                type="date"
                min={today}
                {...register('dueDate')}
                className={cn(
                  'pl-10 pr-10',
                  'text-gray-900 dark:text-gray-100',
                  '[&::-webkit-calendar-picker-indicator]:dark:invert'
                )}
                aria-invalid={!!errors.dueDate}
              />
              {currentDueDate && (
                <button
                  type="button"
                  onClick={() => setValue('dueDate', '')}
                  className="absolute right-3 top-1/2 -translate-y-1/2 p-1 rounded-full hover:bg-gray-100 dark:hover:bg-gray-800"
                >
                  <X className="h-3 w-3 text-gray-500 dark:text-gray-400" />
                </button>
              )}
            </div>
            {errors.dueDate && (
              <p className="text-sm text-destructive">{errors.dueDate.message}</p>
            )}
          </div>

          {/* Priority selection */}
          <div className="space-y-2">
            <Label htmlFor="priority">Priority</Label>
            <div className="flex space-x-2">
              {(['low', 'medium', 'high'] as const).map((priority) => (
                <button
                  key={priority}
                  type="button"
                  className={`flex-1 py-2 px-3 rounded-md text-sm font-medium transition-colors ${
                    watch('priority') === priority
                      ? priority === 'low'
                        ? 'bg-green-500 text-white'
                        : priority === 'medium'
                        ? 'bg-amber-500 text-white'
                        : 'bg-red-500 text-white'
                      : 'bg-gray-100 dark:bg-gray-800 text-gray-700 dark:text-gray-300 hover:bg-gray-200 dark:hover:bg-gray-700'
                  }`}
                  onClick={() => setValue('priority', priority)}
                >
                  {priority.charAt(0).toUpperCase() + priority.slice(1)}
                </button>
              ))}
            </div>
          </div>

          {/* Tags input */}
          <div className="space-y-2">
            <Label htmlFor="tags">Tags (comma separated)</Label>
            <Input
              id="tags"
              placeholder="work, personal, urgent..."
              value={watch('tags')?.join(', ') || ''}
              onChange={(e) => {
                const tags = e.target.value
                  .split(',')
                  .map(tag => tag.trim())
                  .filter(tag => tag.length > 0);
                setValue('tags', tags);
              }}
            />
          </div>

          {/* Recurring task option (create mode only) */}
          {!isEditMode && (
            <div className="flex items-center space-x-2">
              <Checkbox
                id="isRecurring"
                checked={!!watch('isRecurring')}
                onCheckedChange={(checked) => setValue('isRecurring', checked as boolean)}
              />
              <Label htmlFor="isRecurring" className="cursor-pointer">
                Recurring task
              </Label>
            </div>
          )}

          {/* T057: Status Checkbox (edit mode only) */}
          {isEditMode && (
            <div className="flex items-center space-x-2">
              <Checkbox
                id="status"
                checked={currentStatus === 'completed'}
                onCheckedChange={(checked) =>
                  setValue('status', checked ? 'completed' : 'pending')
                }
              />
              <Label htmlFor="status" className="cursor-pointer">
                Mark as completed
              </Label>
            </div>
          )}

          <DialogFooter>
            {/* T065: Cancel button */}
            <Button
              type="button"
              variant="outline"
              onClick={handleClose}
              disabled={isLoading}
            >
              Cancel
            </Button>
            {/* T062: LoadingButton with loading state */}
            <LoadingButton type="submit" loading={isLoading}>
              {isEditMode ? 'Save Changes' : 'Create Task'}
            </LoadingButton>
          </DialogFooter>
        </form>
      </DialogContent>
    </Dialog>
  );
}

export default TaskFormModal;
