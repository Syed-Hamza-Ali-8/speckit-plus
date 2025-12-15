/**
 * User types for the frontend
 * Matches backend UserResponse schema
 */

/**
 * User profile data from GET /auth/me
 */
export interface User {
  id: string;
  email: string;
  first_name: string | null;
  last_name: string | null;
  display_name: string;
  avatar_url: string | null;
  theme: 'light' | 'dark' | 'system';
  email_notifications: boolean;
  is_active: boolean;
  created_at: string;
}

/**
 * Profile update request for PATCH /auth/me
 * All fields are optional (partial update)
 */
export interface UserUpdateRequest {
  first_name?: string | null;
  last_name?: string | null;
  avatar_url?: string | null;
}

/**
 * Task statistics computed from user's tasks
 */
export interface TaskStats {
  total: number;
  completed: number;
  pending: number;
  completionRate: number; // percentage 0-100
}
