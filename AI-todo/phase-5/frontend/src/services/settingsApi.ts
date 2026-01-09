import { api, TAG_TYPES } from './api';
import type {
  UserSettings,
  UserSettingsUpdate,
  PasswordChangeRequest,
  MessageResponse,
} from '@/types/settings';

/**
 * Settings API endpoints
 * Handles settings updates, password changes, and account deletion
 */
export const settingsApi = api.injectEndpoints({
  endpoints: (builder) => ({
    /**
     * Update user settings
     * PATCH /auth/me/settings
     * Partial update - only provided fields are changed
     */
    updateSettings: builder.mutation<UserSettings, UserSettingsUpdate>({
      query: (data) => ({
        url: '/auth/me/settings',
        method: 'PATCH',
        body: data,
      }),
      invalidatesTags: [TAG_TYPES.User],
    }),

    /**
     * Change user password
     * POST /auth/change-password
     * Requires current password verification
     */
    changePassword: builder.mutation<MessageResponse, PasswordChangeRequest>({
      query: (data) => ({
        url: '/auth/change-password',
        method: 'POST',
        body: data,
      }),
    }),

    /**
     * Delete user account
     * DELETE /auth/me
     * Permanently removes user and all associated data
     */
    deleteAccount: builder.mutation<void, void>({
      query: () => ({
        url: '/auth/me',
        method: 'DELETE',
      }),
    }),
  }),
});

// Export hooks for use in components
export const {
  useUpdateSettingsMutation,
  useChangePasswordMutation,
  useDeleteAccountMutation,
} = settingsApi;
