import { Search, X } from 'lucide-react';
import { Input } from '@/components/ui/input';
import { Button } from '@/components/ui/button';
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select';
import type { TaskStatus } from '@/types/task';

/**
 * Props for the TaskFilters component
 */
export interface TaskFiltersProps {
  status: 'all' | TaskStatus;
  search: string;
  onStatusChange: (status: 'all' | TaskStatus) => void;
  onSearchChange: (search: string) => void;
  onClear: () => void;
  resultCount: { showing: number; total: number };
  hasActiveFilters: boolean;
}

/**
 * TaskFilters Component
 *
 * Filter controls for task list:
 * - Status dropdown (All, Pending, Completed)
 * - Search input with icon
 * - Result count display
 * - Clear filters button
 */
export function TaskFilters({
  status,
  search,
  onStatusChange,
  onSearchChange,
  onClear,
  resultCount,
  hasActiveFilters,
}: TaskFiltersProps) {
  return (
    <div className="flex flex-col gap-4 sm:flex-row sm:items-center sm:justify-between">
      <div className="flex flex-1 gap-2">
        {/* T042: Status Select dropdown */}
        <Select value={status} onValueChange={(value) => onStatusChange(value as 'all' | TaskStatus)}>
          <SelectTrigger className="w-[140px]">
            <SelectValue placeholder="Status" />
          </SelectTrigger>
          <SelectContent>
            <SelectItem value="all">All</SelectItem>
            <SelectItem value="pending">Pending</SelectItem>
            <SelectItem value="completed">Completed</SelectItem>
          </SelectContent>
        </Select>

        {/* T043: Search Input */}
        <div className="relative flex-1 max-w-sm">
          <Search className="absolute left-3 top-1/2 -translate-y-1/2 h-4 w-4 text-muted-foreground" />
          <Input
            type="text"
            placeholder="Search tasks..."
            value={search}
            onChange={(e) => onSearchChange(e.target.value)}
            className="pl-9"
          />
        </div>

        {/* T045: Clear filters button */}
        {hasActiveFilters && (
          <Button
            variant="ghost"
            size="icon"
            onClick={onClear}
            className="shrink-0"
            aria-label="Clear filters"
          >
            <X className="h-4 w-4" />
          </Button>
        )}
      </div>

      {/* T044: Result count display */}
      <div className="text-sm text-muted-foreground">
        Showing {resultCount.showing} of {resultCount.total} tasks
      </div>
    </div>
  );
}

export default TaskFilters;
