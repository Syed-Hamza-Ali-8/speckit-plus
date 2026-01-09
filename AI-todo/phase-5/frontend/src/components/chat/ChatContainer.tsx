import { forwardRef, type HTMLAttributes, type ReactNode } from 'react';
import { cn } from '@/lib/utils';

export interface ChatContainerProps extends HTMLAttributes<HTMLDivElement> {
  children: ReactNode;
}

/**
 * ChatContainer - Main wrapper for the chat interface
 * Features glassmorphism styling with gradient background
 */
const ChatContainer = forwardRef<HTMLDivElement, ChatContainerProps>(
  ({ className, children, ...props }, ref) => {
    return (
      <div
        ref={ref}
        className={cn(
          'h-[calc(100vh-4rem)] flex flex-col',
          'bg-gradient-to-br from-purple-900/20 via-blue-900/10 to-indigo-900/20',
          'dark:from-purple-950/30 dark:via-blue-950/20 dark:to-indigo-950/30',
          className
        )}
        {...props}
      >
        <div
          className={cn(
            'flex-1 flex flex-col',
            'backdrop-blur-xl bg-white/10 dark:bg-black/20',
            'rounded-2xl m-4 shadow-xl',
            'border border-white/20 dark:border-white/10',
            'overflow-hidden'
          )}
        >
          {children}
        </div>
      </div>
    );
  }
);

ChatContainer.displayName = 'ChatContainer';

export { ChatContainer };
