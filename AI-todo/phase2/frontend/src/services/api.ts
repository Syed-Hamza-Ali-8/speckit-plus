import {
  createApi,
  fetchBaseQuery,
} from '@reduxjs/toolkit/query/react';
import type {
  BaseQueryFn,
  FetchArgs,
  FetchBaseQueryError,
} from '@reduxjs/toolkit/query/react';

/**
 * RTK Query cache tags for invalidation
 */
export const TAG_TYPES = {
  Task: 'Task',
  User: 'User',
  Notification: 'Notification',
} as const;

export type TagType = (typeof TAG_TYPES)[keyof typeof TAG_TYPES];

/**
 * Base query with auth header injection
 * Reads JWT token from localStorage and attaches to Authorization header
 */
const baseQuery = fetchBaseQuery({
  baseUrl: import.meta.env.VITE_API_URL || '/api',
  prepareHeaders: (headers) => {
    const token = localStorage.getItem('token');
    if (token) {
      headers.set('Authorization', `Bearer ${token}`);
    }
    return headers;
  },
});

/**
 * Base query with 401 response handler
 * On 401 Unauthorized: clears token and redirects to login
 */
const baseQueryWithReauth: BaseQueryFn<
  string | FetchArgs,
  unknown,
  FetchBaseQueryError
> = async (args, api, extraOptions) => {
  const result = await baseQuery(args, api, extraOptions);

  // Handle 401 Unauthorized - token expired or invalid
  if (result.error?.status === 401) {
    // Clear stored token
    localStorage.removeItem('token');
    // Redirect to login page
    window.location.href = '/login';
  }

  return result;
};

/**
 * Base API slice
 * All API endpoints extend from this base
 */
export const api = createApi({
  reducerPath: 'api',
  baseQuery: baseQueryWithReauth,
  tagTypes: Object.values(TAG_TYPES),
  endpoints: () => ({}),
});
