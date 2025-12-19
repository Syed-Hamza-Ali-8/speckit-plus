import {
  forwardRef,
  useState,
  type FormEvent,
  type KeyboardEvent,
  type HTMLAttributes,
} from 'react';
import { cn } from '@/lib/utils';

export interface InputBarProps extends Omit<HTMLAttributes<HTMLFormElement>, 'onSubmit'> {
  onSend: (message: string) => void;
  disabled?: boolean;
  placeholder?: string;
}

/**
 * InputBar - Chat message input with send button
 * Supports Enter to send, Shift+Enter for new line
 */
const InputBar = forwardRef<HTMLFormElement, InputBarProps>(
  (
    {
      className,
      onSend,
      disabled = false,
      placeholder = 'Ask me about your tasks...',
      ...props
    },
    ref
  ) => {
    const [message, setMessage] = useState('');

    const handleSubmit = (e: FormEvent) => {
      e.preventDefault();
      if (message.trim() && !disabled) {
        onSend(message.trim());
        setMessage('');
      }
    };

    const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
      // Enter without Shift sends the message
      if (e.key === 'Enter' && !e.shiftKey) {
        e.preventDefault();
        handleSubmit(e as unknown as FormEvent);
      }
    };

    return (
      <form
        ref={ref}
        onSubmit={handleSubmit}
        className={cn(
          'p-4 border-t border-white/10',
          'bg-white/5 dark:bg-black/10',
          className
        )}
        {...props}
      >
        <div className="flex gap-3 items-end">
          {/* Text input */}
          <div className="flex-1 relative">
            <textarea
              value={message}
              onChange={(e) => setMessage(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder={placeholder}
              disabled={disabled}
              rows={1}
              className={cn(
                'w-full resize-none',
                'bg-white/10 dark:bg-white/5',
                'backdrop-blur-md',
                'border border-white/20 dark:border-white/10',
                'rounded-2xl px-4 py-3 pr-12',
                'text-gray-900 dark:text-gray-100',
                'placeholder-gray-500 dark:placeholder-gray-400',
                'focus:outline-none focus:ring-2 focus:ring-blue-500/50',
                'focus:border-blue-500/50',
                'transition-all duration-200',
                'min-h-[48px] max-h-[120px]',
                disabled && 'opacity-50 cursor-not-allowed'
              )}
              style={{
                height: 'auto',
                minHeight: '48px',
              }}
              onInput={(e) => {
                const target = e.target as HTMLTextAreaElement;
                target.style.height = 'auto';
                target.style.height = `${Math.min(target.scrollHeight, 120)}px`;
              }}
            />
          </div>

          {/* Send button */}
          <button
            type="submit"
            disabled={disabled || !message.trim()}
            className={cn(
              'flex-shrink-0',
              'w-12 h-12 rounded-full',
              'bg-gradient-to-br from-blue-500 to-blue-600',
              'text-white',
              'flex items-center justify-center',
              'shadow-lg shadow-blue-500/30',
              'transition-all duration-200',
              'hover:from-blue-400 hover:to-blue-500',
              'hover:shadow-xl hover:shadow-blue-500/40',
              'hover:scale-105',
              'active:scale-95',
              'disabled:opacity-50 disabled:cursor-not-allowed',
              'disabled:hover:scale-100 disabled:hover:shadow-lg'
            )}
          >
            <svg
              xmlns="http://www.w3.org/2000/svg"
              viewBox="0 0 24 24"
              fill="currentColor"
              className="w-5 h-5"
            >
              <path d="M3.478 2.404a.75.75 0 00-.926.941l2.432 7.905H13.5a.75.75 0 010 1.5H4.984l-2.432 7.905a.75.75 0 00.926.94 60.519 60.519 0 0018.445-8.986.75.75 0 000-1.218A60.517 60.517 0 003.478 2.404z" />
            </svg>
          </button>
        </div>

        {/* Helper text */}
        <p className="text-xs text-gray-500 dark:text-gray-400 mt-2 px-1">
          Press Enter to send, Shift+Enter for new line
        </p>
      </form>
    );
  }
);

InputBar.displayName = 'InputBar';

export { InputBar };
