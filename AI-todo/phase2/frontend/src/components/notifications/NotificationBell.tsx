import { useState, useRef, useEffect } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { Bell } from 'lucide-react';
import { NotificationDropdown } from './NotificationDropdown';
import { useGetUnreadCountQuery } from '@/services/notificationApi';
import { cn } from '@/lib/utils';

export function NotificationBell() {
  const [isOpen, setIsOpen] = useState(false);
  const bellRef = useRef<HTMLButtonElement>(null);

  // Fetch unread count with refetch on window focus
  const { data } = useGetUnreadCountQuery(undefined, {
    pollingInterval: 60000, // Poll every minute
    refetchOnFocus: true,
    refetchOnReconnect: true,
  });

  const unreadCount = data?.unread_count ?? 0;

  // Close dropdown when clicking outside
  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (bellRef.current && !bellRef.current.contains(event.target as Node)) {
        // Check if click is inside dropdown
        const dropdown = document.getElementById('notification-dropdown');
        if (dropdown && dropdown.contains(event.target as Node)) {
          return;
        }
        setIsOpen(false);
      }
    }

    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
      return () => document.removeEventListener('mousedown', handleClickOutside);
    }
  }, [isOpen]);

  // Format badge count (cap at 9+)
  const badgeText = unreadCount > 9 ? '9+' : unreadCount.toString();

  return (
    <div className="relative">
      <motion.button
        ref={bellRef}
        whileHover={{ scale: 1.05 }}
        whileTap={{ scale: 0.95 }}
        onClick={() => setIsOpen(!isOpen)}
        className={cn(
          'relative p-2 rounded-xl',
          'bg-white/70 dark:bg-gray-800/70',
          'backdrop-blur-xl',
          'border border-white/30 dark:border-white/10',
          'shadow-glass dark:shadow-glass-dark',
          'hover:bg-white/90 dark:hover:bg-gray-800/90',
          'transition-colors duration-200',
          isOpen && 'ring-2 ring-purple-500/50'
        )}
        aria-label="Notifications"
        aria-expanded={isOpen}
        aria-haspopup="true"
      >
        <Bell className={cn(
          'h-5 w-5',
          isOpen ? 'text-purple-500' : 'text-muted-foreground'
        )} />

        {/* Notification badge - only show when count > 0 */}
        <AnimatePresence>
          {unreadCount > 0 && (
            <motion.span
              initial={{ scale: 0, opacity: 0 }}
              animate={{ scale: 1, opacity: 1 }}
              exit={{ scale: 0, opacity: 0 }}
              transition={{ type: 'spring', stiffness: 500, damping: 25 }}
              className={cn(
                'absolute -top-1 -right-1',
                'min-w-[18px] h-[18px] flex items-center justify-center',
                'px-1 rounded-full',
                'bg-gradient-to-br from-purple-500 to-indigo-600',
                'text-[10px] font-bold text-white',
                'animate-pulse-glow'
              )}
            >
              {badgeText}
            </motion.span>
          )}
        </AnimatePresence>
      </motion.button>

      {/* Dropdown */}
      <AnimatePresence>
        {isOpen && (
          <NotificationDropdown onClose={() => setIsOpen(false)} />
        )}
      </AnimatePresence>
    </div>
  );
}
