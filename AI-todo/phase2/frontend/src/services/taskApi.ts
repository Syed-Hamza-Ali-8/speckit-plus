import { api, TAG_TYPES } from './api';
import type {
  Task,
  TaskCreate,
  TaskUpdate,
  TaskQueryParams,
  PaginatedResponse,
  RecurringTaskPattern,
  RecurringTaskPatternCreate,
  SetTaskPriorityRequest,
  AddTaskTagsRequest,
  RemoveTaskTagsRequest,
  TaskSearchParams,
  PriorityLevel,
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
     * Also invalidates Notification cache since task creation with due date creates a notification
     */
    createTask: builder.mutation<Task, TaskCreate>({
      query: (task) => ({
        url: '/tasks',
        method: 'POST',
        body: task,
      }),
      invalidatesTags: [
        { type: TAG_TYPES.Task, id: 'LIST' },
        TAG_TYPES.Notification,  // Invalidate notifications (task with due date creates notification)
      ],
    }),

    /**
     * Update existing task
     * PATCH /tasks/:id
     * Also invalidates Notification cache since completing a task creates a notification
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
        TAG_TYPES.Notification,  // Invalidate notifications (completing task creates notification)
      ],
    }),

    /**
     * Delete task
     * DELETE /tasks/:id
     * Also invalidates Notification cache since deleting a task creates a notification
     */
    deleteTask: builder.mutation<void, string>({
      query: (id) => ({
        url: `/tasks/${id}`,
        method: 'DELETE',
      }),
      invalidatesTags: (_result, _error, id) => [
        { type: TAG_TYPES.Task, id },
        { type: TAG_TYPES.Task, id: 'LIST' },
        TAG_TYPES.Notification,  // Invalidate notifications (deleting task creates notification)
      ],
    }),

    /**
     * Phase V: Set task priority
     * PUT /tasks/:id/priority
     */
    setTaskPriority: builder.mutation<Task, { id: string; priority: PriorityLevel }>({
      query: ({ id, priority }) => ({
        url: `/tasks/${id}/priority`,
        method: 'PUT',
        body: { priority },
      }),
      invalidatesTags: (_result, _error, { id }) => [
        { type: TAG_TYPES.Task, id },
        { type: TAG_TYPES.Task, id: 'LIST' },
      ],
    }),

    /**
     * Phase V: Add tags to task
     * PUT /tasks/:id/tags
     */
    addTaskTags: builder.mutation<Task, { id: string; tags: string[] }>({
      query: ({ id, tags }) => ({
        url: `/tasks/${id}/tags`,
        method: 'PUT',
        body: { tags },
      }),
      invalidatesTags: (_result, _error, { id }) => [
        { type: TAG_TYPES.Task, id },
        { type: TAG_TYPES.Task, id: 'LIST' },
      ],
    }),

    /**
     * Phase V: Remove tags from task
     * DELETE /tasks/:id/tags
     */
    removeTaskTags: builder.mutation<Task, { id: string; tags: string[] }>({
      query: ({ id, tags }) => ({
        url: `/tasks/${id}/tags`,
        method: 'DELETE',
        body: { tags },
      }),
      invalidatesTags: (_result, _error, { id }) => [
        { type: TAG_TYPES.Task, id },
        { type: TAG_TYPES.Task, id: 'LIST' },
      ],
    }),

    /**
     * Phase V: Search tasks
     * GET /users/:userId/tasks/search
     */
    searchTasks: builder.query<PaginatedResponse<Task>, TaskSearchParams>({
      query: (params) => {
        const queryParams = new URLSearchParams();
        queryParams.set('query', params.query);
        if (params.status) queryParams.set('status', params.status);
        if (params.priority) queryParams.set('priority', params.priority);
        if (params.tags) params.tags.forEach(tag => queryParams.append('tags', tag));
        if (params.due_before) queryParams.set('due_before', params.due_before);
        if (params.due_after) queryParams.set('due_after', params.due_after);
        if (params.sort_by) queryParams.set('sort_by', params.sort_by);
        if (params.order) queryParams.set('order', params.order);
        if (params.page) queryParams.set('page', params.page.toString());
        if (params.per_page) queryParams.set('per_page', params.per_page.toString());

        return `/users/me/tasks/search?${queryParams.toString()}`;
      },
      providesTags: (result) =>
        result
          ? [
              ...result.items.map(({ id }) => ({ type: TAG_TYPES.Task, id }) as const),
              { type: TAG_TYPES.Task, id: 'SEARCH' },
            ]
          : [{ type: TAG_TYPES.Task, id: 'SEARCH' }],
    }),

    /**
     * Phase V: Get user's recurring task patterns
     * GET /users/:userId/recurring-tasks
     */
    getRecurringTaskPatterns: builder.query<RecurringTaskPattern[], void>({
      query: () => `/users/me/recurring-tasks`,
      providesTags: (result) =>
        result
          ? [
              ...result.map(({ id }) => ({ type: TAG_TYPES.RecurringTaskPattern, id }) as const),
              { type: TAG_TYPES.RecurringTaskPattern, id: 'LIST' },
            ]
          : [{ type: TAG_TYPES.RecurringTaskPattern, id: 'LIST' }],
    }),

    /**
     * Phase V: Get single recurring task pattern by ID
     * GET /recurring-tasks/:id
     */
    getRecurringTaskPattern: builder.query<RecurringTaskPattern, string>({
      query: (id) => `/recurring-tasks/${id}`,
      providesTags: (_result, _error, id) => [{ type: TAG_TYPES.RecurringTaskPattern, id }],
    }),

    /**
     * Phase V: Create recurring task pattern
     * POST /recurring-tasks
     */
    createRecurringTaskPattern: builder.mutation<RecurringTaskPattern, RecurringTaskPatternCreate>({
      query: (pattern) => ({
        url: '/recurring-tasks',
        method: 'POST',
        body: pattern,
      }),
      invalidatesTags: [
        { type: TAG_TYPES.RecurringTaskPattern, id: 'LIST' },
      ],
    }),

    /**
     * Phase V: Update recurring task pattern
     * PUT /recurring-tasks/:id
     */
    updateRecurringTaskPattern: builder.mutation<RecurringTaskPattern, { id: string; data: Partial<RecurringTaskPatternCreate> }>({
      query: ({ id, data }) => ({
        url: `/recurring-tasks/${id}`,
        method: 'PUT',
        body: data,
      }),
      invalidatesTags: (_result, _error, { id }) => [
        { type: TAG_TYPES.RecurringTaskPattern, id },
        { type: TAG_TYPES.RecurringTaskPattern, id: 'LIST' },
      ],
    }),

    /**
     * Phase V: Delete recurring task pattern
     * DELETE /recurring-tasks/:id
     */
    deleteRecurringTaskPattern: builder.mutation<void, string>({
      query: (id) => ({
        url: `/recurring-tasks/${id}`,
        method: 'DELETE',
      }),
      invalidatesTags: (_result, _error, id) => [
        { type: TAG_TYPES.RecurringTaskPattern, id },
        { type: TAG_TYPES.RecurringTaskPattern, id: 'LIST' },
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
  // Phase V: Advanced features
  useSetTaskPriorityMutation,
  useAddTaskTagsMutation,
  useRemoveTaskTagsMutation,
  useSearchTasksQuery,
  useGetRecurringTaskPatternsQuery,
  useGetRecurringTaskPatternQuery,
  useCreateRecurringTaskPatternMutation,
  useUpdateRecurringTaskPatternMutation,
  useDeleteRecurringTaskPatternMutation,
} = taskApi;
