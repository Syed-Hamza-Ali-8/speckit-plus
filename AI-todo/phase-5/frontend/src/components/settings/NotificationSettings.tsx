import { motion } from 'framer-motion';
import { Mail } from 'lucide-react';
import { toast } from 'sonner';
import { useGetCurrentUserQuery } from '@/services/userApi';
import { useUpdateSettingsMutation } from '@/services/settingsApi';
import { cn } from '@/lib/utils';

/**
 * Notification settings component
 * Toggle for email notifications
 */
export function NotificationSettings() {
  const { data: user } = useGetCurrentUserQuery();
  const [updateSettings, { isLoading }] = useUpdateSettingsMutation();

  const handleToggle = async () => {
    if (!user) return;

    const newValue = !user.email_notifications;

    try {
      await updateSettings({ email_notifications: newValue }).unwrap();
      toast.success(
        newValue
          ? 'Email notifications enabled'
          : 'Email notifications disabled'
      );
    } catch {
      toast.error('Failed to update notification settings');
    }
  };

  return (
    <div className="space-y-4">
      <p className="text-sm text-gray-600 dark:text-gray-400">
        Manage how you receive notifications
      </p>

      {/* Email notifications toggle */}
      <motion.div
        whileHover={{ scale: 1.01 }}
        className={cn(
          'flex items-center justify-between p-4 rounded-xl',
          'bg-white/50 dark:bg-gray-800/50',
          'border border-white/20 dark:border-white/10'
        )}
      >
        <div className="flex items-center gap-3">
          <div
            className={cn(
              'flex items-center justify-center',
              'w-10 h-10 rounded-lg',
              'bg-purple-100 dark:bg-purple-900/30'
            )}
          >
            <Mail className="h-5 w-5 text-purple-600 dark:text-purple-400" />
          </div>
          <div>
            <p className="font-medium text-gray-900 dark:text-gray-100">Email Notifications</p>
            <p className="text-sm text-gray-600 dark:text-gray-400">
              Receive updates about your tasks via email
            </p>
          </div>
        </div>

        {/* Custom toggle switch */}
        <button
          onClick={handleToggle}
          disabled={isLoading || !user}
          className={cn(
            'relative w-12 h-7 rounded-full transition-colors duration-200',
            'focus:outline-none focus:ring-2 focus:ring-purple-500/50 focus:ring-offset-2',
            'disabled:opacity-50 disabled:cursor-not-allowed',
            user?.email_notifications
              ? 'bg-gradient-to-r from-purple-500 to-indigo-600'
              : 'bg-gray-200 dark:bg-gray-700'
          )}
          aria-label="Toggle email notifications"
        >
          <motion.span
            layout
            transition={{ type: 'spring', stiffness: 500, damping: 30 }}
            className={cn(
              'absolute top-1 w-5 h-5 rounded-full bg-white shadow-md',
              user?.email_notifications ? 'left-6' : 'left-1'
            )}
          />
        </button>
      </motion.div>
    </div>
  );
}
