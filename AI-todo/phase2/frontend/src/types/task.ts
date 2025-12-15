/**
 * Task types for the frontend
 * Matches backend API contract from Phase 2
 */

/**
 * Task status enumeration
 */
export type TaskStatus = 'pending' | 'completed';

/**
 * Task entity from backend API
 * Note: user_id is NOT included (hidden by backend)
 */
export interface Task {
  id: string;
  title: string;
  description: string | null;
  status: TaskStatus;
  due_date: string | null;  // ISO 8601 date (YYYY-MM-DD)
  created_at: string;   // ISO 8601 datetime
  updated_at: string;   // ISO 8601 datetime
}

/**
 * Create task request payload
 */
export interface TaskCreate {
  title: string;
  description?: string;
  due_date?: string;  // ISO 8601 date (YYYY-MM-DD)
}

/**
 * Update task request payload (partial)
 */
export interface TaskUpdate {
  title?: string;
  description?: string;
  status?: TaskStatus;
  due_date?: string | null;  // ISO 8601 date (YYYY-MM-DD), null to clear
}

/**
 * Paginated response wrapper
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  limit: number;
  offset: number;
}

/**
 * Task list query parameters
 */
export interface TaskQueryParams {
  status?: TaskStatus;
  created_after?: string;   // ISO 8601 date
  created_before?: string;  // ISO 8601 date
  sort?: string;            // Format: "field:direction"
  limit?: number;           // Default: 20, Max: 100
  offset?: number;          // Default: 0
}
