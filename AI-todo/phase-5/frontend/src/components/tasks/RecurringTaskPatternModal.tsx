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
import { Button } from '@/components/ui/button';
import { LoadingButton } from '@/components/ui/loading-button';
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from '@/components/ui/select';
import { Checkbox } from '@/components/ui/checkbox';
import { cn } from '@/lib/utils';
import { recurringTaskPatternSchema } from '@/lib/validations/task';
import type { RecurringTaskPatternFormData } from '@/lib/validations/task';
import {
  useCreateRecurringTaskPatternMutation,
  useUpdateRecurringTaskPatternMutation
} from '@/services/taskApi';
import type { RecurringTaskPattern } from '@/types/task';

/**
 * Props for the RecurringTaskPatternModal component
 */
export interface RecurringTaskPatternModalProps {
  open: boolean;
  onClose: () => void;
  pattern?: RecurringTaskPattern; // If provided, edit mode
  onSuccess: () => void;
}

/**
 * RecurringTaskPatternModal Component
 *
 * Modal dialog for creating and editing recurring task patterns.
 * Features:
 * - React Hook Form with Zod validation
 * - Create mode: base task + recurrence pattern
 * - Edit mode: update existing pattern
 * - RTK Query mutations for API calls
 * - Success/error toast notifications
 * - Form reset on close
 */
export function RecurringTaskPatternModal({
  open,
  onClose,
  pattern,
  onSuccess
}: RecurringTaskPatternModalProps) {
  const isEditMode = !!pattern;

  // RTK Query mutations
  const [createPattern, { isLoading: isCreating }] = useCreateRecurringTaskPatternMutation();
  const [updatePattern, { isLoading: isUpdating }] = useUpdateRecurringTaskPatternMutation();

  const isLoading = isCreating || isUpdating;

  // React Hook Form setup with Zod resolver
  const {
    register,
    handleSubmit,
    reset,
    setValue,
    watch,
    formState: { errors },
  } = useForm<RecurringTaskPatternFormData>({
    resolver: zodResolver(recurringTaskPatternSchema),
    defaultValues: {
      base_task_title: '',
      base_task_description: '',
      pattern_type: 'weekly',
      interval: 1,
      start_date: '',
      end_date: '',
      weekdays: [],
      days_of_month: [],
    },
  });

  // Pre-populate form with pattern data in edit mode
  useEffect(() => {
    if (pattern && open) {
      reset({
        base_task_title: pattern.base_task_title,
        base_task_description: pattern.base_task_description || '',
        pattern_type: pattern.pattern_type,
        interval: pattern.interval,
        start_date: pattern.start_date,
        end_date: pattern.end_date || '',
        weekdays: pattern.weekdays || [],
        days_of_month: pattern.days_of_month || [],
      });
    } else if (!open) {
      // Reset form on modal close
      reset({
        base_task_title: '',
        base_task_description: '',
        pattern_type: 'weekly',
        interval: 1,
        start_date: '',
        end_date: '',
        weekdays: [],
        days_of_month: [],
      });
    }
  }, [pattern, open, reset]);

  // Watch fields for conditional rendering
  const currentPatternType = watch('pattern_type');
  const currentWeekdays = watch('weekdays');
  const currentDaysOfMonth = watch('days_of_month');

  // Get today's date in YYYY-MM-DD format for min attribute
  const today = new Date().toISOString().split('T')[0];

  // Form submission handler
  const onSubmit = async (data: RecurringTaskPatternFormData) => {
    try {
      if (isEditMode && pattern) {
        // Update existing pattern
        await updatePattern({
          id: pattern.id,
          data: data,
        }).unwrap();
        toast.success('Recurring task pattern updated!');
      } else {
        // Create new pattern
        await createPattern(data).unwrap();
        toast.success('Recurring task pattern created!');
      }
      onSuccess();
      onClose();
    } catch (error) {
      // Error toast with API error message
      const errorMessage = error instanceof Error ? error.message : 'An error occurred';
      toast.error(errorMessage);
    }
  };

  // Handle modal close
  const handleClose = () => {
    if (!isLoading) {
      onClose();
    }
  };

  // Toggle weekday selection
  const toggleWeekday = (day: number) => {
    const newWeekdays = currentWeekdays.includes(day)
      ? currentWeekdays.filter(d => d !== day)
      : [...currentWeekdays, day];
    setValue('weekdays', newWeekdays.sort((a, b) => a - b));
  };

  // Toggle day of month selection
  const toggleDayOfMonth = (day: number) => {
    const newDays = currentDaysOfMonth.includes(day)
      ? currentDaysOfMonth.filter(d => d !== day)
      : [...currentDaysOfMonth, day];
    setValue('days_of_month', newDays.sort((a, b) => a - b));
  };

  return (
    <Dialog open={open} onOpenChange={(isOpen) => !isOpen && handleClose()}>
      <DialogContent className="sm:max-w-[500px]">
        <DialogHeader>
          <DialogTitle>
            {isEditMode ? 'Edit Recurring Task Pattern' : 'New Recurring Task Pattern'}
          </DialogTitle>
        </DialogHeader>

        <form onSubmit={handleSubmit(onSubmit)} className="space-y-4">
          {/* Base Task Title */}
          <div className="space-y-2">
            <Label htmlFor="base_task_title">Task Title *</Label>
            <Input
              id="base_task_title"
              placeholder="Enter task title..."
              {...register('base_task_title')}
              aria-invalid={!!errors.base_task_title}
            />
            {errors.base_task_title && (
              <p className="text-sm text-destructive">{errors.base_task_title.message}</p>
            )}
          </div>

          {/* Base Task Description */}
          <div className="space-y-2">
            <Label htmlFor="base_task_description">Task Description (optional)</Label>
            <Textarea
              id="base_task_description"
              placeholder="Enter task description..."
              rows={3}
              {...register('base_task_description')}
              aria-invalid={!!errors.base_task_description}
            />
            {errors.base_task_description && (
              <p className="text-sm text-destructive">{errors.base_task_description.message}</p>
            )}
          </div>

          {/* Pattern Type */}
          <div className="space-y-2">
            <Label htmlFor="pattern_type">Pattern Type *</Label>
            <Select
              value={currentPatternType}
              onValueChange={(value) => setValue('pattern_type', value as any)}
            >
              <SelectTrigger>
                <SelectValue placeholder="Select pattern type" />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="daily">Daily</SelectItem>
                <SelectItem value="weekly">Weekly</SelectItem>
                <SelectItem value="monthly">Monthly</SelectItem>
                <SelectItem value="yearly">Yearly</SelectItem>
              </SelectContent>
            </Select>
            {errors.pattern_type && (
              <p className="text-sm text-destructive">{errors.pattern_type.message}</p>
            )}
          </div>

          {/* Interval */}
          <div className="space-y-2">
            <Label htmlFor="interval">Interval *</Label>
            <Input
              id="interval"
              type="number"
              min="1"
              defaultValue="1"
              {...register('interval', { valueAsNumber: true })}
              aria-invalid={!!errors.interval}
            />
            {errors.interval && (
              <p className="text-sm text-destructive">{errors.interval.message}</p>
            )}
            <p className="text-sm text-gray-500 dark:text-gray-400">
              How often to repeat (e.g., every 2 weeks, every 3 months)
            </p>
          </div>

          {/* Start Date */}
          <div className="space-y-2">
            <Label htmlFor="start_date">Start Date *</Label>
            <div className="relative">
              <CalendarIcon className="absolute left-3 top-1/2 -translate-y-1/2 h-4 w-4 text-gray-500 dark:text-gray-400" />
              <Input
                id="start_date"
                type="date"
                min={today}
                {...register('start_date')}
                className={cn(
                  'pl-10 pr-10',
                  'text-gray-900 dark:text-gray-100',
                  '[&::-webkit-calendar-picker-indicator]:dark:invert'
                )}
                aria-invalid={!!errors.start_date}
              />
            </div>
            {errors.start_date && (
              <p className="text-sm text-destructive">{errors.start_date.message}</p>
            )}
          </div>

          {/* End Date */}
          <div className="space-y-2">
            <Label htmlFor="end_date">End Date (optional)</Label>
            <div className="relative">
              <CalendarIcon className="absolute left-3 top-1/2 -translate-y-1/2 h-4 w-4 text-gray-500 dark:text-gray-400" />
              <Input
                id="end_date"
                type="date"
                min={watch('start_date') || today}
                {...register('end_date')}
                className={cn(
                  'pl-10 pr-10',
                  'text-gray-900 dark:text-gray-100',
                  '[&::-webkit-calendar-picker-indicator]:dark:invert'
                )}
                aria-invalid={!!errors.end_date}
              />
              {watch('end_date') && (
                <button
                  type="button"
                  onClick={() => setValue('end_date', '')}
                  className="absolute right-3 top-1/2 -translate-y-1/2 p-1 rounded-full hover:bg-gray-100 dark:hover:bg-gray-800"
                >
                  <X className="h-3 w-3 text-gray-500 dark:text-gray-400" />
                </button>
              )}
            </div>
            {errors.end_date && (
              <p className="text-sm text-destructive">{errors.end_date.message}</p>
            )}
          </div>

          {/* Weekdays selection for weekly patterns */}
          {currentPatternType === 'weekly' && (
            <div className="space-y-2">
              <Label>Days of Week</Label>
              <div className="grid grid-cols-7 gap-2">
                {['Sun', 'Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat'].map((day, index) => (
                  <div key={index} className="flex items-center space-x-1">
                    <Checkbox
                      id={`weekday-${index}`}
                      checked={currentWeekdays.includes(index)}
                      onCheckedChange={() => toggleWeekday(index)}
                    />
                    <Label htmlFor={`weekday-${index}`} className="text-xs">{day}</Label>
                  </div>
                ))}
              </div>
            </div>
          )}

          {/* Days of month selection for monthly patterns */}
          {currentPatternType === 'monthly' && (
            <div className="space-y-2">
              <Label>Days of Month</Label>
              <div className="grid grid-cols-7 gap-1">
                {Array.from({ length: 31 }, (_, i) => i + 1).map(day => (
                  <div key={day} className="flex items-center space-x-1">
                    <Checkbox
                      id={`day-${day}`}
                      checked={currentDaysOfMonth.includes(day)}
                      onCheckedChange={() => toggleDayOfMonth(day)}
                    />
                    <Label htmlFor={`day-${day}`} className="text-xs">{day}</Label>
                  </div>
                ))}
              </div>
            </div>
          )}

          <DialogFooter>
            <Button
              type="button"
              variant="outline"
              onClick={handleClose}
              disabled={isLoading}
            >
              Cancel
            </Button>
            <LoadingButton type="submit" loading={isLoading}>
              {isEditMode ? 'Save Changes' : 'Create Pattern'}
            </LoadingButton>
          </DialogFooter>
        </form>
      </DialogContent>
    </Dialog>
  );
}

export default RecurringTaskPatternModal;