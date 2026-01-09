/**
 * Task types for the frontend
 * Matches backend API contract from Phase 2 + Phase V advanced features
 */

/**
 * Task status enumeration
 */
export type TaskStatus = 'pending' | 'completed';

/**
 * Priority levels for tasks
 */
export type PriorityLevel = 'low' | 'medium' | 'high';

/**
 * Recurrence pattern types
 */
export type RecurrencePattern = 'daily' | 'weekly' | 'monthly' | 'yearly' | 'custom';

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
  // Phase V: Advanced features
  priority: PriorityLevel;  // Task priority level
  tags: string[];  // List of tags for the task
  is_recurring: boolean;  // Whether this task is part of a recurring pattern
  recurring_pattern_id: string | null;  // ID of the recurring pattern this task belongs to
  is_reminder_sent: boolean;  // Whether a reminder has been sent
  parent_task_id: string | null;  // ID of the parent task (for recurring tasks)
  next_occurrence_id: string | null;  // ID of the next occurrence (for recurring tasks)
}

/**
 * Create task request payload
 */
export interface TaskCreate {
  title: string;
  description?: string;
  due_date?: string;  // ISO 8601 date (YYYY-MM-DD)
  // Phase V: Advanced features
  priority?: PriorityLevel;  // Task priority level
  tags?: string[];  // List of tags for the task
  is_recurring?: boolean;  // Whether this task is part of a recurring pattern
  recurring_pattern_id?: string;  // ID of the recurring pattern this task belongs to
}

/**
 * Update task request payload (partial)
 */
export interface TaskUpdate {
  title?: string;
  description?: string;
  status?: TaskStatus;
  due_date?: string | null;  // ISO 8601 date (YYYY-MM-DD), null to clear
  // Phase V: Advanced features
  priority?: PriorityLevel;  // Task priority level
  tags?: string[];  // List of tags for the task
}

/**
 * Recurring task pattern entity
 */
export interface RecurringTaskPattern {
  id: string;
  user_id: string;
  base_task_title: string;
  base_task_description: string | null;
  pattern_type: RecurrencePattern;
  interval: number;
  start_date: string;  // ISO 8601 date (YYYY-MM-DD)
  end_date: string | null;  // ISO 8601 date (YYYY-MM-DD)
  weekdays: number[];  // List of weekdays for weekly patterns (0=Sunday, 6=Saturday)
  days_of_month: number[];  // List of days of month for monthly patterns
  created_at: string;   // ISO 8601 datetime
  updated_at: string;   // ISO 8601 datetime
}

/**
 * Create recurring task pattern request payload
 */
export interface RecurringTaskPatternCreate {
  base_task_title: string;
  base_task_description?: string;
  pattern_type: RecurrencePattern;
  interval?: number;
  start_date: string;  // ISO 8601 date (YYYY-MM-DD)
  end_date?: string;  // ISO 8601 date (YYYY-MM-DD)
  weekdays?: number[];  // List of weekdays for weekly patterns (0=Sunday, 6=Saturday)
  days_of_month?: number[];  // List of days of month for monthly patterns
}

/**
 * Set task priority request payload
 */
export interface SetTaskPriorityRequest {
  priority: PriorityLevel;
}

/**
 * Add task tags request payload
 */
export interface AddTaskTagsRequest {
  tags: string[];
}

/**
 * Remove task tags request payload
 */
export interface RemoveTaskTagsRequest {
  tags: string[];
}

/**
 * Search tasks request parameters
 */
export interface TaskSearchParams {
  query: string;
  status?: 'all' | 'pending' | 'completed';
  priority?: PriorityLevel;
  tags?: string[];
  due_before?: string;   // ISO 8601 date
  due_after?: string;    // ISO 8601 date
  sort_by?: 'created_at' | 'title' | 'due_date' | 'priority';
  order?: 'asc' | 'desc';
  page?: number;
  per_page?: number;
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
