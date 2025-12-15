/**
 * Notification types
 */
export type NotificationType =
  | 'task_due'
  | 'task_overdue'
  | 'task_completed'
  | 'welcome'
  | 'system';

/**
 * Notification from backend
 */
export interface Notification {
  id: string;
  type: NotificationType;
  title: string;
  message: string;
  is_read: boolean;
  action_url: string | null;
  created_at: string;
}

/**
 * Paginated notification list response
 */
export interface NotificationListResponse {
  notifications: Notification[];
  unread_count: number;
  total: number;
}

/**
 * Mark as read response
 */
export interface NotificationMarkReadResponse {
  id: string;
  is_read: boolean;
}

/**
 * Mark all as read response
 */
export interface NotificationMarkAllReadResponse {
  marked_count: number;
}

/**
 * Unread count response
 */
export interface UnreadCountResponse {
  unread_count: number;
}

/**
 * Query params for notification list
 */
export interface NotificationQueryParams {
  limit?: number;
  offset?: number;
  unread_only?: boolean;
}
