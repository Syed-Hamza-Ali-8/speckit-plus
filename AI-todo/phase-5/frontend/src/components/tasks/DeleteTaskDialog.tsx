import { toast } from 'sonner';
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogDescription,
  DialogFooter,
} from '@/components/ui/dialog';
import { Button } from '@/components/ui/button';
import { LoadingButton } from '@/components/ui/loading-button';
import { useDeleteTaskMutation } from '@/services/taskApi';
import type { Task } from '@/types/task';

/**
 * Props for the DeleteTaskDialog component
 */
export interface DeleteTaskDialogProps {
  open: boolean;
  onClose: () => void;
  task: Task | null;
  onSuccess: () => void;
}

/**
 * DeleteTaskDialog Component
 *
 * Confirmation dialog for deleting a task.
 * Features:
 * - Task title interpolated in confirmation message
 * - Cancel and Delete buttons
 * - Loading state during deletion
 * - Success/error toast notifications
 */
export function DeleteTaskDialog({ open, onClose, task, onSuccess }: DeleteTaskDialogProps) {
  // T072: Integrate useDeleteTaskMutation
  const [deleteTask, { isLoading }] = useDeleteTaskMutation();

  // Handle delete confirmation
  const handleDelete = async () => {
    if (!task) return;

    try {
      await deleteTask(task.id).unwrap();
      // T074: Success toast
      toast.success('Task deleted');
      onSuccess();
      // T076: Close dialog on successful delete
      onClose();
    } catch (error) {
      // T075: Error toast with API error message
      const errorMessage = error instanceof Error ? error.message : 'Failed to delete task';
      toast.error(errorMessage);
    }
  };

  // T076: Handle cancel
  const handleCancel = () => {
    if (!isLoading) {
      onClose();
    }
  };

  return (
    <Dialog open={open} onOpenChange={(isOpen) => !isOpen && handleCancel()}>
      <DialogContent className="sm:max-w-[425px]">
        <DialogHeader>
          {/* T068: Dialog title */}
          <DialogTitle>Delete Task</DialogTitle>
          {/* T069: Confirmation message with task title */}
          <DialogDescription>
            Are you sure you want to delete "{task?.title}"? This action cannot be undone.
          </DialogDescription>
        </DialogHeader>

        <DialogFooter>
          {/* T070: Cancel button (secondary variant) */}
          <Button
            type="button"
            variant="outline"
            onClick={handleCancel}
            disabled={isLoading}
          >
            Cancel
          </Button>
          {/* T071 & T073: Delete button (destructive) with loading state */}
          <LoadingButton
            variant="destructive"
            onClick={handleDelete}
            loading={isLoading}
          >
            Delete
          </LoadingButton>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  );
}

export default DeleteTaskDialog;
