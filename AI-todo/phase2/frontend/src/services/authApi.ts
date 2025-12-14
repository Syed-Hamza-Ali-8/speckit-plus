import { api } from './api';
import type { LoginRequest, RegisterRequest, AuthResponse } from '@/types/auth';

/**
 * Auth API endpoints
 * Handles login and registration mutations
 */
export const authApi = api.injectEndpoints({
  endpoints: (builder) => ({
    /**
     * Login mutation
     * POST /auth/login
     * Returns JWT access token on success
     */
    login: builder.mutation<AuthResponse, LoginRequest>({
      query: (credentials) => ({
        url: '/auth/login',
        method: 'POST',
        body: credentials,
      }),
    }),

    /**
     * Register mutation
     * POST /auth/register
     * Creates new user and returns JWT access token
     */
    register: builder.mutation<AuthResponse, RegisterRequest>({
      query: (data) => ({
        url: '/auth/register',
        method: 'POST',
        body: data,
      }),
    }),
  }),
});

// Export hooks for use in components
export const { useLoginMutation, useRegisterMutation } = authApi;
