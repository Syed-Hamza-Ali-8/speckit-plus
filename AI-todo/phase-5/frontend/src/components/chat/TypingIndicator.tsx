import { forwardRef, type HTMLAttributes } from 'react';
import { cn } from '@/lib/utils';

export interface TypingIndicatorProps extends HTMLAttributes<HTMLDivElement> {}

/**
 * TypingIndicator - Three-dot bouncing animation for loading state
 */
const TypingIndicator = forwardRef<HTMLDivElement, TypingIndicatorProps>(
  ({ className, ...props }, ref) => {
    return (
      <div
        ref={ref}
        className={cn('flex justify-start mb-4', className)}
        {...props}
      >
        <div
          className={cn(
            'flex items-center gap-1',
            'bg-white/20 dark:bg-white/10',
            'backdrop-blur-md',
            'rounded-2xl rounded-bl-md',
            'px-4 py-3',
            'border border-white/20 dark:border-white/10'
          )}
        >
          <span className="sr-only">AI is typing...</span>
          {[0, 1, 2].map((index) => (
            <span
              key={index}
              className={cn(
                'w-2 h-2 rounded-full',
                'bg-gray-500 dark:bg-gray-400',
                'animate-bounce'
              )}
              style={{
                animationDelay: `${index * 150}ms`,
                animationDuration: '600ms',
              }}
            />
          ))}
        </div>
      </div>
    );
  }
);

TypingIndicator.displayName = 'TypingIndicator';

export { TypingIndicator };
