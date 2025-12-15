import { api, TAG_TYPES } from './api';
import type { User, UserUpdateRequest } from '@/types/user';

/**
 * User API endpoints
 * Handles profile fetching and updating
 */
export const userApi = api.injectEndpoints({
  endpoints: (builder) => ({
    /**
     * Get current user profile
     * GET /auth/me
     * Returns authenticated user's profile data
     */
    getCurrentUser: builder.query<User, void>({
      query: () => '/auth/me',
      providesTags: [TAG_TYPES.User],
    }),

    /**
     * Update current user profile
     * PATCH /auth/me
     * Partial update - only provided fields are changed
     */
    updateProfile: builder.mutation<User, UserUpdateRequest>({
      query: (data) => ({
        url: '/auth/me',
        method: 'PATCH',
        body: data,
      }),
      invalidatesTags: [TAG_TYPES.User],
    }),
  }),
});

// Export hooks for use in components
export const { useGetCurrentUserQuery, useUpdateProfileMutation } = userApi;
