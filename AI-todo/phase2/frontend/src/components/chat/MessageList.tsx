import { forwardRef, useEffect, useRef, type HTMLAttributes } from 'react';
import { cn } from '@/lib/utils';
import type { ChatMessage } from '@/types/chat';
import { MessageBubble } from './MessageBubble';
import { TypingIndicator } from './TypingIndicator';

export interface MessageListProps extends HTMLAttributes<HTMLDivElement> {
  messages: ChatMessage[];
  isLoading?: boolean;
}

/**
 * MessageList - Scrollable list of chat messages with auto-scroll
 */
const MessageList = forwardRef<HTMLDivElement, MessageListProps>(
  ({ className, messages, isLoading = false, ...props }, ref) => {
    const bottomRef = useRef<HTMLDivElement>(null);

    // Auto-scroll to bottom when new messages arrive
    useEffect(() => {
      bottomRef.current?.scrollIntoView({ behavior: 'smooth' });
    }, [messages, isLoading]);

    return (
      <div
        ref={ref}
        className={cn(
          'flex-1 overflow-y-auto px-4 py-6',
          'scrollbar-thin scrollbar-thumb-white/20 scrollbar-track-transparent',
          className
        )}
        {...props}
      >
        {/* Empty state */}
        {messages.length === 0 && !isLoading && (
          <div className="flex flex-col items-center justify-center h-full text-center">
            <div className="text-4xl mb-4">
              <span role="img" aria-label="chat">
                <svg
                  xmlns="http://www.w3.org/2000/svg"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  strokeWidth="1.5"
                  className="w-16 h-16 text-gray-400 dark:text-gray-500"
                >
                  <path strokeLinecap="round" strokeLinejoin="round" d="M8.625 12a.375.375 0 11-.75 0 .375.375 0 01.75 0zm0 0H8.25m4.125 0a.375.375 0 11-.75 0 .375.375 0 01.75 0zm0 0H12m4.125 0a.375.375 0 11-.75 0 .375.375 0 01.75 0zm0 0h-.375M21 12c0 4.556-4.03 8.25-9 8.25a9.764 9.764 0 01-2.555-.337A5.972 5.972 0 015.41 20.97a5.969 5.969 0 01-.474-.065 4.48 4.48 0 00.978-2.025c.09-.457-.133-.901-.467-1.226C3.93 16.178 3 14.189 3 12c0-4.556 4.03-8.25 9-8.25s9 3.694 9 8.25z" />
                </svg>
              </span>
            </div>
            <h3 className="text-lg font-medium text-gray-700 dark:text-gray-300 mb-2">
              Start a conversation
            </h3>
            <p className="text-sm text-gray-500 dark:text-gray-400 max-w-sm">
              Ask me about your tasks. I can help you create, update, complete, or
              plan your todo items.
            </p>
            <div className="mt-6 space-y-2 text-sm text-gray-500 dark:text-gray-400">
              <p>"Show my tasks"</p>
              <p>"Add a task to buy groceries"</p>
              <p>"Help me plan my day"</p>
            </div>
          </div>
        )}

        {/* Message list */}
        {messages.map((message, index) => (
          <MessageBubble key={`${message.role}-${index}`} message={message} />
        ))}

        {/* Typing indicator */}
        {isLoading && <TypingIndicator />}

        {/* Scroll anchor */}
        <div ref={bottomRef} />
      </div>
    );
  }
);

MessageList.displayName = 'MessageList';

export { MessageList };
