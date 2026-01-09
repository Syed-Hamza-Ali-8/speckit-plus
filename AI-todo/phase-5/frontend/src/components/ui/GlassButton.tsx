import { forwardRef, type ButtonHTMLAttributes } from 'react';
import { motion, type HTMLMotionProps } from 'framer-motion';
import { cn } from '@/lib/utils';

export interface GlassButtonProps
  extends Omit<ButtonHTMLAttributes<HTMLButtonElement>, 'onAnimationStart' | 'onDrag' | 'onDragEnd' | 'onDragStart'> {
  variant?: 'default' | 'premium' | 'ghost' | 'outline';
  size?: 'sm' | 'md' | 'lg';
  isLoading?: boolean;
  leftIcon?: React.ReactNode;
  rightIcon?: React.ReactNode;
  glow?: boolean;
}

const sizeClasses = {
  sm: 'px-3 py-1.5 text-xs',
  md: 'px-4 py-2 text-sm',
  lg: 'px-6 py-3 text-base',
};

const variantClasses = {
  default: `
    bg-white/70 dark:bg-gray-900/70
    backdrop-blur-xl
    border border-white/30 dark:border-white/10
    text-foreground
    shadow-glass dark:shadow-glass-dark
    hover:bg-white/80 dark:hover:bg-gray-900/80
    hover:border-purple-300/50 dark:hover:border-purple-500/30
  `,
  premium: `
    bg-gradient-to-r from-purple-500 via-blue-500 to-indigo-600
    text-white font-medium
    shadow-premium
    hover:shadow-premium-hover
    border-0
  `,
  ghost: `
    bg-transparent
    hover:bg-white/50 dark:hover:bg-gray-800/50
    backdrop-blur-sm
    text-foreground
  `,
  outline: `
    bg-transparent
    border-2 border-purple-500/50 dark:border-purple-400/50
    text-purple-600 dark:text-purple-400
    hover:bg-purple-500/10 dark:hover:bg-purple-400/10
    backdrop-blur-sm
  `,
};

const GlassButton = forwardRef<HTMLButtonElement, GlassButtonProps>(
  (
    {
      className,
      variant = 'default',
      size = 'md',
      isLoading = false,
      leftIcon,
      rightIcon,
      glow = false,
      children,
      disabled,
      ...props
    },
    ref
  ) => {
    const motionProps: HTMLMotionProps<'button'> = {
      whileHover: disabled || isLoading ? {} : { scale: 1.02, y: -2 },
      whileTap: disabled || isLoading ? {} : { scale: 0.98 },
      transition: { type: 'spring', stiffness: 400, damping: 25 },
    };

    return (
      <motion.button
        ref={ref}
        className={cn(
          'relative inline-flex items-center justify-center gap-2',
          'rounded-xl font-medium',
          'transition-all duration-300 ease-premium',
          'focus:outline-none focus-visible:ring-2 focus-visible:ring-purple-500 focus-visible:ring-offset-2',
          'disabled:opacity-50 disabled:cursor-not-allowed disabled:pointer-events-none',
          sizeClasses[size],
          variantClasses[variant],
          glow && variant === 'premium' && 'animate-pulse-glow',
          className
        )}
        disabled={disabled || isLoading}
        {...motionProps}
        {...(props as HTMLMotionProps<'button'>)}
      >
        {/* Shine effect overlay for premium variant */}
        {variant === 'premium' && (
          <span className="absolute inset-0 overflow-hidden rounded-xl">
            <span className="absolute inset-0 -translate-x-full animate-[shimmer_2s_infinite] bg-gradient-to-r from-transparent via-white/20 to-transparent" />
          </span>
        )}

        {/* Loading spinner */}
        {isLoading && (
          <svg
            className="animate-spin h-4 w-4"
            xmlns="http://www.w3.org/2000/svg"
            fill="none"
            viewBox="0 0 24 24"
          >
            <circle
              className="opacity-25"
              cx="12"
              cy="12"
              r="10"
              stroke="currentColor"
              strokeWidth="4"
            />
            <path
              className="opacity-75"
              fill="currentColor"
              d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"
            />
          </svg>
        )}

        {/* Left icon */}
        {!isLoading && leftIcon && <span className="flex-shrink-0">{leftIcon}</span>}

        {/* Content */}
        <span className="relative z-10">{children}</span>

        {/* Right icon */}
        {!isLoading && rightIcon && <span className="flex-shrink-0">{rightIcon}</span>}
      </motion.button>
    );
  }
);

GlassButton.displayName = 'GlassButton';

export { GlassButton };
