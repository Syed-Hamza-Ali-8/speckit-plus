import { useEffect } from 'react';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { toast } from 'sonner';
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
    },
  });

  // T059: Pre-populate form with task data in edit mode
  useEffect(() => {
    if (task && open) {
      reset({
        title: task.title,
        description: task.description || '',
        status: task.status,
      });
    } else if (!open) {
      // T066: Reset form on modal close
      reset({
        title: '',
        description: '',
        status: 'pending',
      });
    }
  }, [task, open, reset]);

  // Watch status for checkbox
  const currentStatus = watch('status');

  // Form submission handler
  const onSubmit = async (data: TaskFormData | TaskEditFormData) => {
    try {
      if (isEditMode && task) {
        // T061: Update existing task - only send changed fields
        const updateData: { title?: string; description?: string; status?: 'pending' | 'completed' } = {};

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
