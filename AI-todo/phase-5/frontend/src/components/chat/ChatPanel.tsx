import { useCallback } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { useDispatch } from 'react-redux';
import { cn } from '@/lib/utils';
import { MessageList } from './MessageList';
import { InputBar } from './InputBar';
import { useChat } from '@/hooks/useChat';
import { api, TAG_TYPES } from '@/services/api';

export interface ChatPanelProps {
  isOpen: boolean;
  onClose: () => void;
}

/**
 * ChatPanel - Expandable chat panel with glassmorphism design
 *
 * Features:
 * - Slide-up animation from bottom-right
 * - Glassmorphism styling
 * - Full chat functionality
 * - Close button in header
 * - Auto-refreshes task list when tasks are modified via chat
 */
export function ChatPanel({ isOpen, onClose }: ChatPanelProps) {
  const dispatch = useDispatch();

  // Callback to invalidate tasks cache when chat modifies tasks
  const handleTasksChanged = useCallback(() => {
    dispatch(api.util.invalidateTags([{ type: TAG_TYPES.Task, id: 'LIST' }]));
  }, [dispatch]);

  const { messages, isLoading, sendMessage, clearMessages } = useChat(handleTasksChanged);

  return (
    <AnimatePresence>
      {isOpen && (
        <>
          {/* Backdrop for mobile */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            onClick={onClose}
            className="fixed inset-0 bg-black/20 backdrop-blur-sm z-40 md:hidden"
          />

          {/* Chat Panel */}
          <motion.div
            initial={{ opacity: 0, y: 100, scale: 0.9 }}
            animate={{ opacity: 1, y: 0, scale: 1 }}
            exit={{ opacity: 0, y: 100, scale: 0.9 }}
            transition={{ type: 'spring', damping: 25, stiffness: 300 }}
            className={cn(
              'fixed z-50',
              // Mobile: full screen with safe margins (top has more space for browser UI)
              'top-16 bottom-4 left-4 right-4',
              'md:inset-auto',
              // Desktop: bottom-right corner with fixed size
              'md:bottom-6 md:right-6',
              'md:w-[400px] md:h-[550px]',
              // Styling
              'flex flex-col',
              'bg-white/80 dark:bg-gray-900/90',
              'backdrop-blur-xl',
              'rounded-2xl',
              'shadow-2xl shadow-black/20',
              'border border-white/30 dark:border-white/10',
              'overflow-hidden'
            )}
          >
            {/* Header */}
            <div
              className={cn(
                'flex items-center justify-between',
                'px-4 py-3',
                'border-b border-white/20 dark:border-white/10',
                'bg-gradient-to-r from-blue-500/10 to-purple-500/10'
              )}
            >
              <div className="flex items-center gap-3">
                <div
                  className={cn(
                    'w-10 h-10 rounded-full',
                    'bg-gradient-to-br from-blue-500 to-purple-600',
                    'flex items-center justify-center'
                  )}
                >
                  <svg
                    xmlns="http://www.w3.org/2000/svg"
                    viewBox="0 0 24 24"
                    fill="currentColor"
                    className="w-5 h-5 text-white"
                  >
                    <path d="M4.913 2.658c2.075-.27 4.19-.408 6.337-.408 2.147 0 4.262.139 6.337.408 1.922.25 3.291 1.861 3.405 3.727a4.403 4.403 0 00-1.032-.211 50.89 50.89 0 00-8.42 0c-2.358.196-4.04 2.19-4.04 4.434v4.286a4.47 4.47 0 002.433 3.984L7.28 21.53A.75.75 0 016 21v-4.03a48.527 48.527 0 01-1.087-.128C2.905 16.58 1.5 14.833 1.5 12.862V6.638c0-1.97 1.405-3.718 3.413-3.979z" />
                    <path d="M15.75 7.5c-1.376 0-2.739.057-4.086.169C10.124 7.797 9 9.103 9 10.609v4.285c0 1.507 1.128 2.814 2.67 2.94 1.243.102 2.5.157 3.768.165l2.782 2.781a.75.75 0 001.28-.53v-2.39l.33-.026c1.542-.125 2.67-1.433 2.67-2.94v-4.286c0-1.505-1.125-2.811-2.664-2.94A49.392 49.392 0 0015.75 7.5z" />
                  </svg>
                </div>
                <div>
                  <h2 className="text-sm font-semibold text-gray-900 dark:text-white">
                    Todo AI Assistant
                  </h2>
                  <p className="text-xs text-gray-500 dark:text-gray-400">
                    {isLoading ? 'Thinking...' : 'Ask me anything'}
                  </p>
                </div>
              </div>

              <div className="flex items-center gap-2">
                {/* Clear chat button */}
                {messages.length > 0 && (
                  <button
                    onClick={clearMessages}
                    className={cn(
                      'p-2 rounded-lg',
                      'text-gray-500 hover:text-gray-700',
                      'dark:text-gray-400 dark:hover:text-gray-200',
                      'hover:bg-gray-100 dark:hover:bg-gray-800',
                      'transition-colors'
                    )}
                    title="Clear chat"
                  >
                    <svg
                      xmlns="http://www.w3.org/2000/svg"
                      viewBox="0 0 20 20"
                      fill="currentColor"
                      className="w-5 h-5"
                    >
                      <path
                        fillRule="evenodd"
                        d="M8.75 1A2.75 2.75 0 006 3.75v.443c-.795.077-1.584.176-2.365.298a.75.75 0 10.23 1.482l.149-.022.841 10.518A2.75 2.75 0 007.596 19h4.807a2.75 2.75 0 002.742-2.53l.841-10.519.149.023a.75.75 0 00.23-1.482A41.03 41.03 0 0014 4.193V3.75A2.75 2.75 0 0011.25 1h-2.5zM10 4c.84 0 1.673.025 2.5.075V3.75c0-.69-.56-1.25-1.25-1.25h-2.5c-.69 0-1.25.56-1.25 1.25v.325C8.327 4.025 9.16 4 10 4zM8.58 7.72a.75.75 0 00-1.5.06l.3 7.5a.75.75 0 101.5-.06l-.3-7.5zm4.34.06a.75.75 0 10-1.5-.06l-.3 7.5a.75.75 0 101.5.06l.3-7.5z"
                        clipRule="evenodd"
                      />
                    </svg>
                  </button>
                )}

                {/* Close button */}
                <button
                  onClick={onClose}
                  className={cn(
                    'p-2 rounded-lg',
                    'text-gray-500 hover:text-gray-700',
                    'dark:text-gray-400 dark:hover:text-gray-200',
                    'hover:bg-gray-100 dark:hover:bg-gray-800',
                    'transition-colors'
                  )}
                  aria-label="Close chat"
                >
                  <svg
                    xmlns="http://www.w3.org/2000/svg"
                    viewBox="0 0 20 20"
                    fill="currentColor"
                    className="w-5 h-5"
                  >
                    <path d="M6.28 5.22a.75.75 0 00-1.06 1.06L8.94 10l-3.72 3.72a.75.75 0 101.06 1.06L10 11.06l3.72 3.72a.75.75 0 101.06-1.06L11.06 10l3.72-3.72a.75.75 0 00-1.06-1.06L10 8.94 6.28 5.22z" />
                  </svg>
                </button>
              </div>
            </div>

            {/* Messages */}
            <MessageList messages={messages} isLoading={isLoading} />

            {/* Input */}
            <InputBar onSend={sendMessage} disabled={isLoading} />
          </motion.div>
        </>
      )}
    </AnimatePresence>
  );
}
