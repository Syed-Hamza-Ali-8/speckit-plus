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
import { useDeleteRecurringTaskPatternMutation } from '@/services/taskApi';
import type { RecurringTaskPattern } from '@/types/task';

/**
 * Props for the DeleteRecurringTaskPatternDialog component
 */
export interface DeleteRecurringTaskPatternDialogProps {
  open: boolean;
  onClose: () => void;
  pattern: RecurringTaskPattern | null;
  onSuccess: () => void;
}

/**
 * DeleteRecurringTaskPatternDialog Component
 *
 * Confirmation dialog for deleting a recurring task pattern.
 * Features:
 * - Pattern title interpolated in confirmation message
 * - Cancel and Delete buttons
 * - Loading state during deletion
 * - Success/error toast notifications
 * - Custom message explaining that existing tasks won't be deleted
 */
export function DeleteRecurringTaskPatternDialog({
  open,
  onClose,
  pattern,
  onSuccess
}: DeleteRecurringTaskPatternDialogProps) {
  const [deletePattern, { isLoading }] = useDeleteRecurringTaskPatternMutation();

  // Handle delete confirmation
  const handleDelete = async () => {
    if (!pattern) return;

    try {
      await deletePattern(pattern.id).unwrap();
      toast.success('Recurring task pattern deleted');
      onSuccess();
      onClose();
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Failed to delete recurring task pattern';
      toast.error(errorMessage);
    }
  };

  // Handle cancel
  const handleCancel = () => {
    if (!isLoading) {
      onClose();
    }
  };

  return (
    <Dialog open={open} onOpenChange={(isOpen) => !isOpen && handleCancel()}>
      <DialogContent className="sm:max-w-[425px]">
        <DialogHeader>
          <DialogTitle>Delete Recurring Task Pattern</DialogTitle>
          <DialogDescription>
            Are you sure you want to delete the recurring pattern "{pattern?.title}"?
            <br /><br />
            <strong>Note:</strong> This will not delete existing tasks that were generated from this pattern.
          </DialogDescription>
        </DialogHeader>

        <DialogFooter>
          <Button
            type="button"
            variant="outline"
            onClick={handleCancel}
            disabled={isLoading}
          >
            Cancel
          </Button>
          <LoadingButton
            variant="destructive"
            onClick={handleDelete}
            loading={isLoading}
          >
            Delete Pattern
          </LoadingButton>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  );
}

export default DeleteRecurringTaskPatternDialog;
