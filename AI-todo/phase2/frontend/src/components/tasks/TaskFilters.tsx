import { useState } from 'react';
import { motion } from 'framer-motion';
import { Search, X, Filter, ListFilter } from 'lucide-react';
import { GlassCard } from '@/components/ui/GlassCard';
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select';
import { cn } from '@/lib/utils';
import type { TaskStatus } from '@/types/task';

export interface TaskFiltersProps {
  status: 'all' | TaskStatus;
  search: string;
  onStatusChange: (status: 'all' | TaskStatus) => void;
  onSearchChange: (search: string) => void;
  onClear: () => void;
  resultCount: { showing: number; total: number };
  hasActiveFilters: boolean;
}

export function TaskFilters({
  status,
  search,
  onStatusChange,
  onSearchChange,
  onClear,
  resultCount,
  hasActiveFilters,
}: TaskFiltersProps) {
  const [isSearchFocused, setIsSearchFocused] = useState(false);

  return (
    <GlassCard variant="default" hover={false} padding="md">
      <div className="flex flex-col gap-4 sm:flex-row sm:items-center sm:justify-between">
        <div className="flex flex-1 gap-3 items-center">
          {/* Filter icon */}
          <div
            className={cn(
              'hidden sm:flex items-center justify-center',
              'w-10 h-10 rounded-xl',
              'bg-gradient-to-br from-purple-500/10 to-indigo-500/10',
              'border border-purple-500/20'
            )}
          >
            <Filter className="h-4 w-4 text-purple-500" />
          </div>

          {/* Status Select dropdown */}
          <Select
            value={status}
            onValueChange={(value) => onStatusChange(value as 'all' | TaskStatus)}
          >
            <SelectTrigger
              className={cn(
                'w-[140px] rounded-xl',
                'bg-white/50 dark:bg-gray-800/50',
                'border-white/30 dark:border-white/10',
                'hover:border-purple-300/50 dark:hover:border-purple-500/30',
                'focus:ring-2 focus:ring-purple-500/20',
                'transition-all duration-200',
                'text-gray-900 dark:text-gray-100'
              )}
            >
              <ListFilter className="h-4 w-4 mr-2 text-gray-500 dark:text-gray-400" />
              <SelectValue placeholder="Status" />
            </SelectTrigger>
            <SelectContent
              className={cn(
                'bg-white/90 dark:bg-gray-900/90',
                'backdrop-blur-xl',
                'border border-white/30 dark:border-white/10',
                'shadow-xl rounded-xl',
                'text-gray-900 dark:text-gray-100'
              )}
            >
              <SelectItem value="all" className="rounded-lg hover:bg-purple-500/10 text-gray-900 dark:text-gray-100">
                All
              </SelectItem>
              <SelectItem value="pending" className="rounded-lg hover:bg-purple-500/10 text-gray-900 dark:text-gray-100">
                Pending
              </SelectItem>
              <SelectItem value="completed" className="rounded-lg hover:bg-purple-500/10 text-gray-900 dark:text-gray-100">
                Completed
              </SelectItem>
            </SelectContent>
          </Select>

          {/* Search Input */}
          <div
            className={cn(
              'relative flex-1 max-w-sm',
              'transition-all duration-300',
              isSearchFocused && 'scale-[1.02]'
            )}
          >
            <Search
              className={cn(
                'absolute left-3 top-1/2 -translate-y-1/2 h-4 w-4',
                'transition-colors duration-200',
                isSearchFocused ? 'text-purple-500' : 'text-gray-500 dark:text-gray-400'
              )}
            />
            <input
              type="text"
              placeholder="Search tasks..."
              value={search}
              onChange={(e) => onSearchChange(e.target.value)}
              onFocus={() => setIsSearchFocused(true)}
              onBlur={() => setIsSearchFocused(false)}
              className={cn(
                'w-full h-10 pl-10 pr-4',
                'rounded-xl',
                'bg-white/50 dark:bg-gray-800/50',
                'border border-white/30 dark:border-white/10',
                'text-sm text-gray-900 dark:text-gray-100 placeholder:text-gray-500 dark:placeholder:text-gray-400',
                'transition-all duration-200',
                'focus:outline-none focus:ring-2 focus:ring-purple-500/20',
                'focus:border-purple-300/50 dark:focus:border-purple-500/30',
                isSearchFocused && 'shadow-glow'
              )}
            />
          </div>

          {/* Clear filters button */}
          {hasActiveFilters && (
            <motion.button
              initial={{ opacity: 0, scale: 0.8 }}
              animate={{ opacity: 1, scale: 1 }}
              exit={{ opacity: 0, scale: 0.8 }}
              whileHover={{ scale: 1.1 }}
              whileTap={{ scale: 0.9 }}
              onClick={onClear}
              className={cn(
                'p-2 rounded-lg',
                'bg-red-500/10 dark:bg-red-500/20',
                'text-red-600 dark:text-red-400',
                'hover:bg-red-500/20 dark:hover:bg-red-500/30',
                'transition-colors duration-200'
              )}
              aria-label="Clear filters"
            >
              <X className="h-4 w-4" />
            </motion.button>
          )}
        </div>

        {/* Result count display */}
        <motion.div
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          className={cn(
            'text-sm px-3 py-1.5 rounded-lg',
            'bg-purple-500/10 dark:bg-purple-500/20',
            'text-purple-600 dark:text-purple-400',
            'font-medium'
          )}
        >
          {resultCount.showing === resultCount.total ? (
            <span>{resultCount.total} tasks</span>
          ) : (
            <span>
              {resultCount.showing} of {resultCount.total}
            </span>
          )}
        </motion.div>
      </div>
    </GlassCard>
  );
}

export default TaskFilters;
