import { api, TAG_TYPES } from './api';
import type {
  Task,
  TaskCreate,
  TaskUpdate,
  TaskQueryParams,
  PaginatedResponse,
} from '@/types/task';

/**
 * Task API endpoints
 * Handles CRUD operations for tasks with cache invalidation
 */
export const taskApi = api.injectEndpoints({
  endpoints: (builder) => ({
    /**
     * Get tasks with pagination and filtering
     * GET /tasks
     */
    getTasks: builder.query<PaginatedResponse<Task>, TaskQueryParams | void>({
      query: (params) => {
        const queryParams = new URLSearchParams();
        if (params) {
          if (params.status) queryParams.set('status', params.status);
          if (params.created_after) queryParams.set('created_after', params.created_after);
          if (params.created_before) queryParams.set('created_before', params.created_before);
          if (params.sort) queryParams.set('sort', params.sort);
          if (params.limit) queryParams.set('limit', params.limit.toString());
          if (params.offset) queryParams.set('offset', params.offset.toString());
        }
        const queryString = queryParams.toString();
        return `/tasks${queryString ? `?${queryString}` : ''}`;
      },
      providesTags: (result) =>
        result
          ? [
              ...result.items.map(({ id }) => ({ type: TAG_TYPES.Task, id }) as const),
              { type: TAG_TYPES.Task, id: 'LIST' },
            ]
          : [{ type: TAG_TYPES.Task, id: 'LIST' }],
    }),

    /**
     * Get single task by ID
     * GET /tasks/:id
     */
    getTask: builder.query<Task, string>({
      query: (id) => `/tasks/${id}`,
      providesTags: (_result, _error, id) => [{ type: TAG_TYPES.Task, id }],
    }),

    /**
     * Create new task
     * POST /tasks
     */
    createTask: builder.mutation<Task, TaskCreate>({
      query: (task) => ({
        url: '/tasks',
        method: 'POST',
        body: task,
      }),
      invalidatesTags: [{ type: TAG_TYPES.Task, id: 'LIST' }],
    }),

    /**
     * Update existing task
     * PATCH /tasks/:id
     */
    updateTask: builder.mutation<Task, { id: string; data: TaskUpdate }>({
      query: ({ id, data }) => ({
        url: `/tasks/${id}`,
        method: 'PATCH',
        body: data,
      }),
      invalidatesTags: (_result, _error, { id }) => [
        { type: TAG_TYPES.Task, id },
        { type: TAG_TYPES.Task, id: 'LIST' },
      ],
    }),

    /**
     * Delete task
     * DELETE /tasks/:id
     */
    deleteTask: builder.mutation<void, string>({
      query: (id) => ({
        url: `/tasks/${id}`,
        method: 'DELETE',
      }),
      invalidatesTags: (_result, _error, id) => [
        { type: TAG_TYPES.Task, id },
        { type: TAG_TYPES.Task, id: 'LIST' },
      ],
    }),
  }),
});

// Export hooks for use in components
export const {
  useGetTasksQuery,
  useGetTaskQuery,
  useCreateTaskMutation,
  useUpdateTaskMutation,
  useDeleteTaskMutation,
} = taskApi;
