/**
 * Settings types for the frontend
 * Matches backend settings schemas
 */

/**
 * Theme options
 */
export type Theme = 'light' | 'dark' | 'system';

/**
 * User settings from backend
 */
export interface UserSettings {
  theme: Theme;
  email_notifications: boolean;
}

/**
 * Settings update request for PATCH /auth/me/settings
 * All fields are optional (partial update)
 */
export interface UserSettingsUpdate {
  theme?: Theme;
  email_notifications?: boolean;
}

/**
 * Password change request for POST /auth/change-password
 */
export interface PasswordChangeRequest {
  current_password: string;
  new_password: string;
}

/**
 * Generic message response
 */
export interface MessageResponse {
  message: string;
}
