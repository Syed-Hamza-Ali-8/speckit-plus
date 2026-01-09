import { useState } from 'react';
import { Link, useLocation, useNavigate } from 'react-router-dom';
import { motion, AnimatePresence } from 'framer-motion';
import {
  CheckSquare,
  Search,
  User,
  LogOut,
  Settings,
  ChevronDown,
} from 'lucide-react';
import { AnimatedThemeToggle } from './AnimatedThemeToggle';
import { GlassButton } from '@/components/ui/GlassButton';
import { NotificationBell } from '@/components/notifications';
import { useAuth } from '@/hooks/useAuth';
import { useGetCurrentUserQuery } from '@/services/userApi';
import { ROUTES } from '@/routes';
import { cn } from '@/lib/utils';

export interface HeaderProps {
  onMenuClick?: () => void;
}

export function Header(_props: HeaderProps) {
  const { isAuthenticated, logout } = useAuth();
  // Fetch real user data from API
  const { data: user } = useGetCurrentUserQuery(undefined, {
    skip: !isAuthenticated,
  });
  const location = useLocation();
  const navigate = useNavigate();
  const [isProfileOpen, setIsProfileOpen] = useState(false);
  const [isSearchFocused, setIsSearchFocused] = useState(false);

  // Use real user data or fallback
  const displayName = user?.name || user?.email?.split('@')[0] || 'User';
  const userEmail = user?.email || 'user@example.com';
  const avatarInitial = user?.name?.charAt(0) || user?.email?.charAt(0) || 'U';

  const isAuthPage =
    location.pathname === ROUTES.LOGIN || location.pathname === ROUTES.REGISTER;

  return (
    <motion.header
      initial={{ y: -20, opacity: 0 }}
      animate={{ y: 0, opacity: 1 }}
      transition={{ duration: 0.3, ease: 'easeOut' }}
      className={cn(
        'sticky top-0 z-50 w-full',
        'glass-header'
      )}
    >
      <div className="container mx-auto flex h-16 items-center justify-between px-4 lg:px-6">
        {/* Logo */}
        <Link
          to={ROUTES.HOME}
          className="flex items-center gap-2.5 group"
        >
          <motion.div
            whileHover={{ rotate: [0, -10, 10, 0] }}
            transition={{ duration: 0.5 }}
            className={cn(
              'flex items-center justify-center',
              'w-9 h-9 rounded-xl',
              'bg-gradient-to-br from-purple-500 via-blue-500 to-indigo-600',
              'shadow-premium'
            )}
          >
            <CheckSquare className="h-5 w-5 text-white" />
          </motion.div>
          <span className="font-bold text-lg tracking-tight">
            <span className="text-gradient">Task</span>
            <span className="text-foreground">GPT</span>
          </span>
        </Link>

        {/* Center - Search (only when authenticated) */}
        {isAuthenticated && (
          <motion.div
            initial={{ opacity: 0, scale: 0.9 }}
            animate={{ opacity: 1, scale: 1 }}
            transition={{ delay: 0.1 }}
            className="hidden md:flex flex-1 max-w-md mx-8"
          >
            <div
              className={cn(
                'relative w-full',
                'transition-all duration-300',
                isSearchFocused && 'scale-105'
              )}
            >
              <Search
                className={cn(
                  'absolute left-3 top-1/2 -translate-y-1/2 h-4 w-4',
                  'transition-colors duration-200',
                  isSearchFocused
                    ? 'text-purple-500'
                    : 'text-muted-foreground'
                )}
              />
              <input
                type="search"
                placeholder="Search tasks..."
                onFocus={() => setIsSearchFocused(true)}
                onBlur={() => setIsSearchFocused(false)}
                className={cn(
                  'w-full h-10 pl-10 pr-4',
                  'rounded-xl',
                  'bg-white/70 dark:bg-gray-900/70',
                  'backdrop-blur-xl',
                  'border border-white/30 dark:border-white/10',
                  'shadow-glass dark:shadow-glass-dark',
                  'text-sm text-foreground placeholder:text-muted-foreground',
                  'transition-all duration-300',
                  'focus:outline-none focus:ring-2 focus:ring-purple-500/50',
                  'focus:border-purple-300/50 dark:focus:border-purple-500/30',
                  isSearchFocused && 'shadow-glow'
                )}
              />
              <kbd
                className={cn(
                  'absolute right-3 top-1/2 -translate-y-1/2',
                  'hidden lg:inline-flex items-center gap-1',
                  'px-1.5 py-0.5 rounded',
                  'bg-white/50 dark:bg-gray-800/50',
                  'border border-white/30 dark:border-white/10',
                  'text-xs text-muted-foreground',
                  'font-mono'
                )}
              >
                <span className="text-[10px]">⌘</span>K
              </kbd>
            </div>
          </motion.div>
        )}

        {/* Right side - Navigation */}
        <nav className="flex items-center gap-2">
          {!isAuthenticated && !isAuthPage && (
            <motion.div
              initial={{ opacity: 0, x: 20 }}
              animate={{ opacity: 1, x: 0 }}
              transition={{ delay: 0.2 }}
              className="flex items-center gap-2"
            >
              <Link to={ROUTES.LOGIN}>
                <GlassButton variant="ghost" size="sm">
                  Login
                </GlassButton>
              </Link>
              <Link to={ROUTES.REGISTER}>
                <GlassButton variant="premium" size="sm">
                  Get Started
                </GlassButton>
              </Link>
            </motion.div>
          )}

          {isAuthenticated && (
            <motion.div
              initial={{ opacity: 0, x: 20 }}
              animate={{ opacity: 1, x: 0 }}
              transition={{ delay: 0.2 }}
              className="flex items-center gap-2"
            >
              {/* Notifications */}
              <NotificationBell />

              {/* Profile Dropdown */}
              <div className="relative">
                <motion.button
                  whileHover={{ scale: 1.02 }}
                  whileTap={{ scale: 0.98 }}
                  onClick={() => setIsProfileOpen(!isProfileOpen)}
                  className={cn(
                    'flex items-center gap-2 p-1.5 pr-3 rounded-xl',
                    'bg-white/70 dark:bg-gray-800/70',
                    'backdrop-blur-xl',
                    'border border-white/30 dark:border-white/10',
                    'shadow-glass dark:shadow-glass-dark',
                    'hover:bg-white/90 dark:hover:bg-gray-800/90',
                    'transition-all duration-200'
                  )}
                >
                  <div
                    className={cn(
                      'w-7 h-7 rounded-lg',
                      'bg-gradient-to-br from-purple-500 via-blue-500 to-indigo-600',
                      'flex items-center justify-center',
                      'text-xs font-bold text-white'
                    )}
                  >
                    {avatarInitial.toUpperCase()}
                  </div>
                  <span className="hidden sm:block text-sm font-medium text-foreground max-w-[100px] truncate">
                    {displayName}
                  </span>
                  <motion.div
                    animate={{ rotate: isProfileOpen ? 180 : 0 }}
                    transition={{ duration: 0.2 }}
                  >
                    <ChevronDown className="h-4 w-4 text-muted-foreground" />
                  </motion.div>
                </motion.button>

                {/* Dropdown Menu */}
                <AnimatePresence>
                  {isProfileOpen && (
                    <>
                      {/* Backdrop */}
                      <motion.div
                        initial={{ opacity: 0 }}
                        animate={{ opacity: 1 }}
                        exit={{ opacity: 0 }}
                        className="fixed inset-0 z-40"
                        onClick={() => setIsProfileOpen(false)}
                      />

                      <motion.div
                        initial={{ opacity: 0, y: 10, scale: 0.95 }}
                        animate={{ opacity: 1, y: 0, scale: 1 }}
                        exit={{ opacity: 0, y: 10, scale: 0.95 }}
                        transition={{
                          type: 'spring',
                          stiffness: 300,
                          damping: 25,
                        }}
                        className={cn(
                          'absolute right-0 mt-2 w-56 z-50',
                          'rounded-xl overflow-hidden',
                          'bg-white/90 dark:bg-gray-900/90',
                          'backdrop-blur-xl',
                          'border border-white/30 dark:border-white/10',
                          'shadow-xl shadow-black/10 dark:shadow-black/30'
                        )}
                      >
                        {/* User info */}
                        <div className="px-4 py-3 border-b border-white/20 dark:border-white/10">
                          <p className="text-sm font-medium text-foreground">
                            {displayName}
                          </p>
                          <p className="text-xs text-muted-foreground truncate">
                            {userEmail}
                          </p>
                        </div>

                        {/* Menu items */}
                        <div className="py-1">
                          <motion.button
                            whileHover={{ x: 4 }}
                            onClick={() => {
                              setIsProfileOpen(false);
                              navigate(ROUTES.PROFILE);
                            }}
                            className={cn(
                              'flex items-center gap-3 w-full px-4 py-2.5',
                              'text-sm text-foreground',
                              'hover:bg-purple-500/10',
                              'transition-colors duration-150'
                            )}
                          >
                            <User className="h-4 w-4 text-muted-foreground" />
                            Profile
                          </motion.button>
                          <motion.button
                            whileHover={{ x: 4 }}
                            onClick={() => {
                              setIsProfileOpen(false);
                              navigate(ROUTES.SETTINGS);
                            }}
                            className={cn(
                              'flex items-center gap-3 w-full px-4 py-2.5',
                              'text-sm text-foreground',
                              'hover:bg-purple-500/10',
                              'transition-colors duration-150'
                            )}
                          >
                            <Settings className="h-4 w-4 text-muted-foreground" />
                            Settings
                          </motion.button>
                          <motion.button
                            whileHover={{ x: 4 }}
                            onClick={() => {
                              setIsProfileOpen(false);
                              navigate(ROUTES.RECURRING_PATTERNS);
                            }}
                            className={cn(
                              'flex items-center gap-3 w-full px-4 py-2.5',
                              'text-sm text-foreground',
                              'hover:bg-purple-500/10',
                              'transition-colors duration-150'
                            )}
                          >
                            <CheckSquare className="h-4 w-4 text-muted-foreground" />
                            Recurring Patterns
                          </motion.button>
                        </div>

                        {/* Logout */}
                        <div className="border-t border-white/20 dark:border-white/10 py-1">
                          <motion.button
                            whileHover={{ x: 4 }}
                            onClick={() => {
                              setIsProfileOpen(false);
                              logout();
                            }}
                            className={cn(
                              'flex items-center gap-3 w-full px-4 py-2.5',
                              'text-sm text-red-600 dark:text-red-400',
                              'hover:bg-red-500/10',
                              'transition-colors duration-150'
                            )}
                          >
                            <LogOut className="h-4 w-4" />
                            Logout
                          </motion.button>
                        </div>
                      </motion.div>
                    </>
                  )}
                </AnimatePresence>
              </div>
            </motion.div>
          )}

          {/* Theme Toggle */}
          <AnimatedThemeToggle size="md" />
        </nav>
      </div>
    </motion.header>
  );
}
