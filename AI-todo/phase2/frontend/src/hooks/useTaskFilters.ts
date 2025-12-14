import { useState, useEffect, useCallback, useMemo } from 'react';
import { useSearchParams } from 'react-router-dom';
import type { TaskStatus } from '@/types/task';

/**
 * Filter state for tasks
 */
export interface TaskFilters {
  status: 'all' | TaskStatus;
  search: string;
}

/**
 * Return type for useTaskFilters hook
 */
export interface UseTaskFiltersReturn {
  filters: TaskFilters;
  setStatus: (status: 'all' | TaskStatus) => void;
  setSearch: (search: string) => void;
  clearFilters: () => void;
  hasActiveFilters: boolean;
  debouncedSearch: string;
}

/**
 * useTaskFilters Hook
 *
 * Manages task filter state with URL synchronization.
 * Features:
 * - Status filter (all/pending/completed)
 * - Search filter with 300ms debounce
 * - URL sync: /tasks?status=pending&search=meeting
 * - Clear all filters function
 */
export function useTaskFilters(): UseTaskFiltersReturn {
  const [searchParams, setSearchParams] = useSearchParams();

  // T037: Status filter state from URL
  const status = (searchParams.get('status') as 'all' | TaskStatus) || 'all';

  // T038: Search filter state from URL
  const search = searchParams.get('search') || '';

  // T039: Debounced search value (300ms)
  const [debouncedSearch, setDebouncedSearch] = useState(search);

  // Debounce search input
  useEffect(() => {
    const timer = setTimeout(() => {
      setDebouncedSearch(search);
    }, 300);

    return () => clearTimeout(timer);
  }, [search]);

  // Update URL params helper
  const updateParams = useCallback(
    (updates: Partial<TaskFilters>) => {
      const newParams = new URLSearchParams(searchParams);

      if (updates.status !== undefined) {
        if (updates.status === 'all') {
          newParams.delete('status');
        } else {
          newParams.set('status', updates.status);
        }
      }

      if (updates.search !== undefined) {
        if (updates.search === '') {
          newParams.delete('search');
        } else {
          newParams.set('search', updates.search);
        }
      }

      setSearchParams(newParams, { replace: true });
    },
    [searchParams, setSearchParams]
  );

  // T037: Set status filter
  const setStatus = useCallback(
    (newStatus: 'all' | TaskStatus) => {
      updateParams({ status: newStatus });
    },
    [updateParams]
  );

  // T038: Set search filter
  const setSearch = useCallback(
    (newSearch: string) => {
      updateParams({ search: newSearch });
    },
    [updateParams]
  );

  // T040: Clear all filters
  const clearFilters = useCallback(() => {
    setSearchParams(new URLSearchParams(), { replace: true });
    setDebouncedSearch('');
  }, [setSearchParams]);

  // Check if any filters are active
  const hasActiveFilters = useMemo(
    () => status !== 'all' || search !== '',
    [status, search]
  );

  // Combined filters object
  const filters = useMemo(
    () => ({
      status,
      search,
    }),
    [status, search]
  );

  return {
    filters,
    setStatus,
    setSearch,
    clearFilters,
    hasActiveFilters,
    debouncedSearch,
  };
}

export default useTaskFilters;
