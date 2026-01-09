import { motion, AnimatePresence } from 'framer-motion';
import { Sun, Moon } from 'lucide-react';
import { useTheme } from '@/hooks/useTheme';
import { cn } from '@/lib/utils';

export interface AnimatedThemeToggleProps {
  className?: string;
  size?: 'sm' | 'md' | 'lg';
}

const sizeClasses = {
  sm: 'h-8 w-8',
  md: 'h-9 w-9',
  lg: 'h-10 w-10',
};

const iconSizeClasses = {
  sm: 'h-4 w-4',
  md: 'h-5 w-5',
  lg: 'h-6 w-6',
};

export function AnimatedThemeToggle({
  className,
  size = 'md',
}: AnimatedThemeToggleProps) {
  const { toggleTheme, isDark } = useTheme();

  return (
    <motion.button
      onClick={toggleTheme}
      className={cn(
        'relative inline-flex items-center justify-center',
        'rounded-xl',
        'bg-white/70 dark:bg-gray-800/70',
        'backdrop-blur-xl',
        'border border-white/30 dark:border-white/10',
        'shadow-glass dark:shadow-glass-dark',
        'hover:bg-white/90 dark:hover:bg-gray-800/90',
        'hover:border-purple-300/50 dark:hover:border-purple-500/30',
        'transition-colors duration-300',
        'focus:outline-none focus-visible:ring-2 focus-visible:ring-purple-500 focus-visible:ring-offset-2',
        sizeClasses[size],
        className
      )}
      whileHover={{ scale: 1.05 }}
      whileTap={{ scale: 0.95 }}
      aria-label={isDark ? 'Switch to light mode' : 'Switch to dark mode'}
    >
      <AnimatePresence mode="wait" initial={false}>
        {isDark ? (
          <motion.div
            key="sun"
            initial={{ rotate: -90, scale: 0, opacity: 0 }}
            animate={{ rotate: 0, scale: 1, opacity: 1 }}
            exit={{ rotate: 90, scale: 0, opacity: 0 }}
            transition={{
              type: 'spring',
              stiffness: 200,
              damping: 15,
            }}
          >
            <Sun
              className={cn(
                iconSizeClasses[size],
                'text-amber-500'
              )}
            />
          </motion.div>
        ) : (
          <motion.div
            key="moon"
            initial={{ rotate: 90, scale: 0, opacity: 0 }}
            animate={{ rotate: 0, scale: 1, opacity: 1 }}
            exit={{ rotate: -90, scale: 0, opacity: 0 }}
            transition={{
              type: 'spring',
              stiffness: 200,
              damping: 15,
            }}
          >
            <Moon
              className={cn(
                iconSizeClasses[size],
                'text-purple-500 dark:text-purple-400'
              )}
            />
          </motion.div>
        )}
      </AnimatePresence>

      {/* Glow effect on hover */}
      <motion.div
        className={cn(
          'absolute inset-0 rounded-xl',
          isDark
            ? 'bg-amber-500/20'
            : 'bg-purple-500/20',
          'opacity-0 group-hover:opacity-100',
          'transition-opacity duration-300'
        )}
        initial={false}
        animate={{
          boxShadow: isDark
            ? '0 0 20px rgba(245, 158, 11, 0.3)'
            : '0 0 20px rgba(168, 85, 247, 0.3)',
        }}
      />
    </motion.button>
  );
}
