import { motion } from 'framer-motion';
import { useNavigate } from 'react-router-dom';
import {
  Clock,
  AlertTriangle,
  CheckCircle,
  Star,
  Info,
} from 'lucide-react';
import { formatDistanceToNow } from 'date-fns';
import { toast } from 'sonner';
import { useMarkAsReadMutation } from '@/services/notificationApi';
import type { Notification, NotificationType } from '@/types/notification';
import { cn } from '@/lib/utils';

interface NotificationItemProps {
  notification: Notification;
  onClose: () => void;
}

// Icon mapping for notification types
const typeIcons: Record<NotificationType, typeof Clock> = {
  task_due: Clock,
  task_overdue: AlertTriangle,
  task_completed: CheckCircle,
  welcome: Star,
  system: Info,
};

// Color mapping for notification types
const typeColors: Record<NotificationType, string> = {
  task_due: 'text-amber-500',
  task_overdue: 'text-red-500',
  task_completed: 'text-green-500',
  welcome: 'text-purple-500',
  system: 'text-blue-500',
};

export function NotificationItem({ notification, onClose }: NotificationItemProps) {
  const navigate = useNavigate();
  const [markAsRead] = useMarkAsReadMutation();

  const Icon = typeIcons[notification.type] || Info;
  const iconColor = typeColors[notification.type] || 'text-muted-foreground';

  // Format relative timestamp
  const timeAgo = formatDistanceToNow(new Date(notification.created_at), {
    addSuffix: true,
  });

  const handleClick = async () => {
    try {
      // Mark as read if not already read
      if (!notification.is_read) {
        await markAsRead(notification.id).unwrap();
      }

      // Navigate to action URL if present
      if (notification.action_url) {
        onClose();
        navigate(notification.action_url);
      }
    } catch {
      toast.error('Failed to mark notification as read');
    }
  };

  return (
    <motion.button
      whileHover={{ x: 4, backgroundColor: 'rgba(139, 92, 246, 0.05)' }}
      onClick={handleClick}
      className={cn(
        'w-full flex items-start gap-3 p-3',
        'text-left',
        'transition-colors duration-150',
        notification.is_read && 'opacity-60'
      )}
    >
      {/* Icon */}
      <div
        className={cn(
          'flex-shrink-0 w-8 h-8 rounded-lg',
          'flex items-center justify-center',
          'bg-white/50 dark:bg-gray-800/50',
          notification.is_read && 'opacity-50'
        )}
      >
        <Icon className={cn('h-4 w-4', iconColor)} />
      </div>

      {/* Content */}
      <div className="flex-1 min-w-0">
        <p
          className={cn(
            'text-sm font-medium text-foreground truncate',
            notification.is_read && 'font-normal'
          )}
        >
          {notification.title}
        </p>
        <p className="text-xs text-muted-foreground line-clamp-2 mt-0.5">
          {notification.message}
        </p>
        <p className="text-[10px] text-muted-foreground mt-1">
          {timeAgo}
        </p>
      </div>

      {/* Unread indicator */}
      {!notification.is_read && (
        <div className="flex-shrink-0 mt-1.5">
          <span
            className={cn(
              'w-2 h-2 rounded-full',
              'bg-gradient-to-br from-purple-500 to-indigo-600',
              'block'
            )}
          />
        </div>
      )}
    </motion.button>
  );
}
