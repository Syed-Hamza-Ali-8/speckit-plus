import { motion } from 'framer-motion';
import { Bell, CheckCheck, Trash2 } from 'lucide-react';
import { toast } from 'sonner';
import { NotificationItem } from './NotificationItem';
import {
  useGetNotificationsQuery,
  useMarkAllAsReadMutation,
  useClearAllNotificationsMutation,
} from '@/services/notificationApi';
import { cn } from '@/lib/utils';

interface NotificationDropdownProps {
  onClose: () => void;
}

export function NotificationDropdown({ onClose }: NotificationDropdownProps) {
  const { data, isLoading } = useGetNotificationsQuery({ limit: 10, offset: 0 });
  const [markAllAsRead, { isLoading: isMarkingAll }] = useMarkAllAsReadMutation();
  const [clearAllNotifications, { isLoading: isClearing }] = useClearAllNotificationsMutation();

  const notifications = data?.notifications ?? [];
  const unreadCount = data?.unread_count ?? 0;
  const totalCount = data?.total ?? 0;

  const handleMarkAllRead = async () => {
    try {
      const result = await markAllAsRead().unwrap();
      if (result.marked_count > 0) {
        toast.success(`Marked ${result.marked_count} notifications as read`);
      }
    } catch {
      toast.error('Failed to mark all as read');
    }
  };

  const handleClearAll = async () => {
    try {
      const result = await clearAllNotifications().unwrap();
      if (result.deleted_count > 0) {
        toast.success(`Cleared ${result.deleted_count} notifications`);
      }
    } catch {
      toast.error('Failed to clear notifications');
    }
  };

  return (
    <motion.div
      id="notification-dropdown"
      initial={{ opacity: 0, y: 10, scale: 0.95 }}
      animate={{ opacity: 1, y: 0, scale: 1 }}
      exit={{ opacity: 0, y: 10, scale: 0.95 }}
      transition={{ type: 'spring', stiffness: 300, damping: 25 }}
      className={cn(
        'absolute right-0 mt-2 w-80 z-50',
        'rounded-xl overflow-hidden',
        'bg-white/95 dark:bg-gray-900/95',
        'backdrop-blur-xl',
        'border border-white/30 dark:border-white/10',
        'shadow-xl shadow-black/10 dark:shadow-black/30'
      )}
    >
      {/* Header */}
      <div
        className={cn(
          'flex items-center justify-between',
          'px-4 py-3',
          'border-b border-white/20 dark:border-white/10'
        )}
      >
        <h3 className="text-sm font-semibold text-gray-900 dark:text-gray-100">
          Notifications
        </h3>
        <div className="flex items-center gap-2">
          {unreadCount > 0 && (
            <motion.button
              whileHover={{ scale: 1.05 }}
              whileTap={{ scale: 0.95 }}
              onClick={handleMarkAllRead}
              disabled={isMarkingAll}
              className={cn(
                'flex items-center gap-1.5',
                'px-2 py-1 rounded-lg',
                'text-xs font-medium',
                'text-purple-600 dark:text-purple-400',
                'hover:bg-purple-500/10',
                'transition-colors duration-150',
                'disabled:opacity-50 disabled:cursor-not-allowed'
              )}
            >
              <CheckCheck className="h-3.5 w-3.5" />
              Mark all read
            </motion.button>
          )}
          {totalCount > 0 && (
            <motion.button
              whileHover={{ scale: 1.05 }}
              whileTap={{ scale: 0.95 }}
              onClick={handleClearAll}
              disabled={isClearing}
              className={cn(
                'flex items-center gap-1.5',
                'px-2 py-1 rounded-lg',
                'text-xs font-medium',
                'text-red-600 dark:text-red-400',
                'hover:bg-red-500/10',
                'transition-colors duration-150',
                'disabled:opacity-50 disabled:cursor-not-allowed'
              )}
            >
              <Trash2 className="h-3.5 w-3.5" />
              Clear all
            </motion.button>
          )}
        </div>
      </div>

      {/* Notification List */}
      <div className="max-h-[360px] overflow-y-auto">
        {isLoading ? (
          <div className="flex items-center justify-center py-8">
            <div className="animate-spin rounded-full h-6 w-6 border-2 border-purple-500 border-t-transparent" />
          </div>
        ) : notifications.length > 0 ? (
          <div className="divide-y divide-white/10 dark:divide-white/5">
            {notifications.map((notification) => (
              <NotificationItem
                key={notification.id}
                notification={notification}
                onClose={onClose}
              />
            ))}
          </div>
        ) : (
          <EmptyState />
        )}
      </div>

      {/* Footer - View All (optional, for future implementation) */}
      {notifications.length > 0 && data && data.total > 10 && (
        <div
          className={cn(
            'px-4 py-2',
            'border-t border-white/20 dark:border-white/10'
          )}
        >
          <button
            className={cn(
              'w-full text-center text-xs font-medium',
              'text-purple-600 dark:text-purple-400',
              'hover:text-purple-700 dark:hover:text-purple-300',
              'transition-colors duration-150'
            )}
          >
            View all {data.total} notifications
          </button>
        </div>
      )}
    </motion.div>
  );
}

function EmptyState() {
  return (
    <div className="flex flex-col items-center justify-center py-12 px-4">
      <div
        className={cn(
          'w-12 h-12 rounded-full',
          'flex items-center justify-center',
          'bg-gradient-to-br from-purple-500/20 to-indigo-500/20',
          'mb-3'
        )}
      >
        <Bell className="h-6 w-6 text-purple-500" />
      </div>
      <p className="text-sm font-medium text-foreground">
        No notifications yet
      </p>
      <p className="text-xs text-muted-foreground mt-1">
        You're all caught up!
      </p>
    </div>
  );
}
