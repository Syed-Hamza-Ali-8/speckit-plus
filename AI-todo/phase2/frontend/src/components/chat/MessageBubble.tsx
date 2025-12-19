import { forwardRef, type HTMLAttributes } from 'react';
import { cn } from '@/lib/utils';
import type { ChatMessage, ActionTaken } from '@/types/chat';
import { ActionChip } from './ActionChip';

export interface MessageBubbleProps extends HTMLAttributes<HTMLDivElement> {
  message: ChatMessage;
}

/**
 * MessageBubble - Individual message display component
 * User messages align right with blue background
 * Assistant messages align left with glassmorphism styling
 */
const MessageBubble = forwardRef<HTMLDivElement, MessageBubbleProps>(
  ({ className, message, ...props }, ref) => {
    const isUser = message.role === 'user';

    return (
      <div
        ref={ref}
        className={cn(
          'flex w-full mb-4',
          isUser ? 'justify-end' : 'justify-start',
          className
        )}
        {...props}
      >
        <div
          className={cn(
            'max-w-[85%] md:max-w-[75%] rounded-2xl px-4 py-3',
            'transition-all duration-200',
            isUser
              ? [
                  'bg-gradient-to-br from-blue-500 to-blue-600',
                  'text-white',
                  'rounded-br-md',
                  'shadow-lg shadow-blue-500/20',
                ]
              : [
                  'bg-white/20 dark:bg-white/10',
                  'backdrop-blur-md',
                  'text-gray-900 dark:text-gray-100',
                  'rounded-bl-md',
                  'border border-white/20 dark:border-white/10',
                  'shadow-lg shadow-black/5 dark:shadow-black/20',
                ]
          )}
        >
          {/* Message content */}
          <p className="whitespace-pre-wrap break-words text-sm md:text-base">
            {message.content}
          </p>

          {/* Action chips for assistant messages */}
          {!isUser && message.actions && message.actions.length > 0 && (
            <div className="flex flex-wrap gap-2 mt-3 pt-3 border-t border-white/10">
              {message.actions.map((action: ActionTaken, index: number) => (
                <ActionChip key={`${action.tool}-${index}`} action={action} />
              ))}
            </div>
          )}

          {/* Timestamp */}
          <div
            className={cn(
              'text-xs mt-2 opacity-70',
              isUser ? 'text-blue-100' : 'text-gray-500 dark:text-gray-400'
            )}
          >
            {new Date(message.timestamp).toLocaleTimeString([], {
              hour: '2-digit',
              minute: '2-digit',
            })}
          </div>
        </div>
      </div>
    );
  }
);

MessageBubble.displayName = 'MessageBubble';

export { MessageBubble };
