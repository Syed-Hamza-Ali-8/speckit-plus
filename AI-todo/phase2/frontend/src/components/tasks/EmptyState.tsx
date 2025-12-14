import { ClipboardList } from 'lucide-react';
import { Button } from '@/components/ui/button';

/**
 * Props for the EmptyState component
 */
export interface EmptyStateProps {
  hasFilters: boolean;
  onCreateClick: () => void;
  onClearFilters: () => void;
}

/**
 * EmptyState Component
 *
 * Displays when no tasks exist or no tasks match filters.
 * Two variants:
 * 1. No tasks at all - shows create CTA
 * 2. No matching tasks - shows clear filters link
 */
export function EmptyState({ hasFilters, onCreateClick, onClearFilters }: EmptyStateProps) {
  return (
    <div className="flex flex-col items-center justify-center py-12 px-4 text-center">
      {/* T022: ClipboardList icon centered */}
      <div className="rounded-full bg-muted p-4 mb-4">
        <ClipboardList className="h-10 w-10 text-muted-foreground" />
      </div>

      {hasFilters ? (
        // T026: Filtered empty state variant
        <>
          <h3 className="text-lg font-semibold mb-2">No tasks match your filters</h3>
          <p className="text-sm text-muted-foreground mb-4">
            Try adjusting your search or filter criteria
          </p>
          <Button variant="link" onClick={onClearFilters} className="text-primary">
            Clear filters
          </Button>
        </>
      ) : (
        // T023, T024, T025: Default empty state
        <>
          <h3 className="text-lg font-semibold mb-2">No tasks yet</h3>
          <p className="text-sm text-muted-foreground mb-4">
            Create your first task to get started
          </p>
          <Button onClick={onCreateClick}>Create Task</Button>
        </>
      )}
    </div>
  );
}

export default EmptyState;
