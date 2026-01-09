import { useEffect } from 'react';
import { motion } from 'framer-motion';
import { Sun, Moon, Monitor } from 'lucide-react';
import { toast } from 'sonner';
import { useTheme } from '@/hooks/useTheme';
import { useGetCurrentUserQuery } from '@/services/userApi';
import { useUpdateSettingsMutation } from '@/services/settingsApi';
import { cn } from '@/lib/utils';
import type { Theme } from '@/stores/uiStore';

const themes: { value: Theme; label: string; icon: typeof Sun }[] = [
  { value: 'light', label: 'Light', icon: Sun },
  { value: 'dark', label: 'Dark', icon: Moon },
  { value: 'system', label: 'System', icon: Monitor },
];

/**
 * Theme settings component
 * Allows users to select light, dark, or system theme
 * Syncs to backend and persists across sessions
 */
export function ThemeSettings() {
  const { theme, setTheme } = useTheme();
  const { data: user } = useGetCurrentUserQuery();
  const [updateSettings, { isLoading }] = useUpdateSettingsMutation();

  // Sync theme from backend on mount
  useEffect(() => {
    if (user?.theme && user.theme !== theme) {
      setTheme(user.theme);
    }
  }, [user?.theme, setTheme, theme]);

  const handleThemeChange = async (newTheme: Theme) => {
    // Optimistic update - apply immediately
    setTheme(newTheme);

    try {
      await updateSettings({ theme: newTheme }).unwrap();
      toast.success('Theme updated');
    } catch {
      // Revert on error
      if (user?.theme) {
        setTheme(user.theme);
      }
      toast.error('Failed to update theme');
    }
  };

  return (
    <div className="space-y-3">
      <p className="text-sm text-gray-600 dark:text-gray-400 mb-4">
        Choose your preferred theme for the interface
      </p>

      <div className="grid grid-cols-3 gap-3">
        {themes.map(({ value, label, icon: Icon }) => (
          <motion.button
            key={value}
            whileHover={{ scale: 1.02 }}
            whileTap={{ scale: 0.98 }}
            onClick={() => handleThemeChange(value)}
            disabled={isLoading}
            className={cn(
              'flex flex-col items-center gap-2 p-4 rounded-xl',
              'border-2 transition-all duration-200',
              'disabled:opacity-50 disabled:cursor-not-allowed',
              theme === value
                ? 'border-purple-500 bg-purple-500/10 shadow-glow'
                : 'border-white/20 dark:border-white/10 hover:border-purple-300/50 bg-white/50 dark:bg-gray-800/50'
            )}
          >
            <Icon
              className={cn(
                'h-6 w-6',
                theme === value
                  ? 'text-purple-500'
                  : 'text-gray-600 dark:text-gray-400'
              )}
            />
            <span
              className={cn(
                'text-sm font-medium',
                theme === value
                  ? 'text-purple-600 dark:text-purple-400'
                  : 'text-gray-900 dark:text-gray-100'
              )}
            >
              {label}
            </span>
          </motion.button>
        ))}
      </div>
    </div>
  );
}
