import { api } from './api';
import type {
  Notification,
  NotificationClearAllResponse,
  NotificationDeleteResponse,
  NotificationListResponse,
  NotificationMarkAllReadResponse,
  NotificationMarkReadResponse,
  NotificationQueryParams,
  UnreadCountResponse,
} from '@/types/notification';

export const notificationApi = api.injectEndpoints({
  endpoints: (builder) => ({
    /**
     * Get paginated notifications for current user
     */
    getNotifications: builder.query<NotificationListResponse, NotificationQueryParams>({
      query: ({ limit = 10, offset = 0, unread_only = false }) =>
        `/notifications?limit=${limit}&offset=${offset}&unread_only=${unread_only}`,
      providesTags: ['Notification'],
    }),

    /**
     * Get unread notification count (lightweight endpoint for badge)
     */
    getUnreadCount: builder.query<UnreadCountResponse, void>({
      query: () => '/notifications/unread-count',
      providesTags: ['Notification'],
    }),

    /**
     * Mark a single notification as read
     * Uses optimistic update with rollback on error
     */
    markAsRead: builder.mutation<NotificationMarkReadResponse, string>({
      query: (id) => ({
        url: `/notifications/${id}/read`,
        method: 'PATCH',
      }),
      // Optimistic update
      async onQueryStarted(id, { dispatch, queryFulfilled }) {
        // Update getNotifications cache optimistically
        const patchNotifications = dispatch(
          notificationApi.util.updateQueryData(
            'getNotifications',
            { limit: 10, offset: 0 },
            (draft) => {
              const notification = draft.notifications.find(
                (n: Notification) => n.id === id
              );
              if (notification && !notification.is_read) {
                notification.is_read = true;
                draft.unread_count = Math.max(0, draft.unread_count - 1);
              }
            }
          )
        );

        // Update unread count cache optimistically
        const patchUnreadCount = dispatch(
          notificationApi.util.updateQueryData('getUnreadCount', undefined, (draft) => {
            draft.unread_count = Math.max(0, draft.unread_count - 1);
          })
        );

        try {
          await queryFulfilled;
        } catch {
          // Rollback on error
          patchNotifications.undo();
          patchUnreadCount.undo();
        }
      },
    }),

    /**
     * Mark all notifications as read
     */
    markAllAsRead: builder.mutation<NotificationMarkAllReadResponse, void>({
      query: () => ({
        url: '/notifications/mark-all-read',
        method: 'POST',
      }),
      // Optimistic update
      async onQueryStarted(_, { dispatch, queryFulfilled }) {
        // Update getNotifications cache optimistically
        const patchNotifications = dispatch(
          notificationApi.util.updateQueryData(
            'getNotifications',
            { limit: 10, offset: 0 },
            (draft) => {
              draft.notifications.forEach((n: Notification) => {
                n.is_read = true;
              });
              draft.unread_count = 0;
            }
          )
        );

        // Update unread count cache optimistically
        const patchUnreadCount = dispatch(
          notificationApi.util.updateQueryData('getUnreadCount', undefined, (draft) => {
            draft.unread_count = 0;
          })
        );

        try {
          await queryFulfilled;
        } catch {
          // Rollback on error
          patchNotifications.undo();
          patchUnreadCount.undo();
        }
      },
    }),

    /**
     * Delete a single notification
     */
    deleteNotification: builder.mutation<NotificationDeleteResponse, string>({
      query: (id) => ({
        url: `/notifications/${id}`,
        method: 'DELETE',
      }),
      // Optimistic update
      async onQueryStarted(id, { dispatch, queryFulfilled }) {
        // Update getNotifications cache optimistically
        const patchNotifications = dispatch(
          notificationApi.util.updateQueryData(
            'getNotifications',
            { limit: 10, offset: 0 },
            (draft) => {
              const index = draft.notifications.findIndex(
                (n: Notification) => n.id === id
              );
              if (index !== -1) {
                const wasUnread = !draft.notifications[index].is_read;
                draft.notifications.splice(index, 1);
                draft.total -= 1;
                if (wasUnread) {
                  draft.unread_count = Math.max(0, draft.unread_count - 1);
                }
              }
            }
          )
        );

        // Update unread count cache - will be refreshed by invalidatesTags
        const patchUnreadCount = dispatch(
          notificationApi.util.updateQueryData('getUnreadCount', undefined, (_draft) => {
            // We don't know if it was unread, so we'll let the invalidation handle it
          })
        );

        try {
          await queryFulfilled;
        } catch {
          // Rollback on error
          patchNotifications.undo();
          patchUnreadCount.undo();
        }
      },
      invalidatesTags: ['Notification'],
    }),

    /**
     * Clear all notifications
     */
    clearAllNotifications: builder.mutation<NotificationClearAllResponse, void>({
      query: () => ({
        url: '/notifications',
        method: 'DELETE',
      }),
      // Optimistic update
      async onQueryStarted(_, { dispatch, queryFulfilled }) {
        // Update getNotifications cache optimistically
        const patchNotifications = dispatch(
          notificationApi.util.updateQueryData(
            'getNotifications',
            { limit: 10, offset: 0 },
            (draft) => {
              draft.notifications = [];
              draft.unread_count = 0;
              draft.total = 0;
            }
          )
        );

        // Update unread count cache optimistically
        const patchUnreadCount = dispatch(
          notificationApi.util.updateQueryData('getUnreadCount', undefined, (draft) => {
            draft.unread_count = 0;
          })
        );

        try {
          await queryFulfilled;
        } catch {
          // Rollback on error
          patchNotifications.undo();
          patchUnreadCount.undo();
        }
      },
      invalidatesTags: ['Notification'],
    }),
  }),
});

export const {
  useGetNotificationsQuery,
  useGetUnreadCountQuery,
  useMarkAsReadMutation,
  useMarkAllAsReadMutation,
  useDeleteNotificationMutation,
  useClearAllNotificationsMutation,
} = notificationApi;
