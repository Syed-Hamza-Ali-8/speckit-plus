import { forwardRef, type HTMLAttributes } from 'react';
import { cn } from '@/lib/utils';

export interface ShimmerSkeletonProps extends HTMLAttributes<HTMLDivElement> {
  variant?: 'default' | 'circular' | 'rounded';
  width?: string | number;
  height?: string | number;
  lines?: number;
  animated?: boolean;
}

const variantClasses = {
  default: 'rounded-md',
  circular: 'rounded-full',
  rounded: 'rounded-xl',
};

const ShimmerSkeleton = forwardRef<HTMLDivElement, ShimmerSkeletonProps>(
  (
    {
      className,
      variant = 'default',
      width,
      height,
      lines,
      animated = true,
      style,
      ...props
    },
    ref
  ) => {
    const baseClasses = cn(
      'bg-gradient-to-r',
      'from-gray-200/60 via-gray-100/80 to-gray-200/60',
      'dark:from-gray-800/60 dark:via-gray-700/80 dark:to-gray-800/60',
      'bg-[length:200%_100%]',
      animated && 'animate-shimmer',
      variantClasses[variant]
    );

    // If lines prop is provided, render multiple skeleton lines
    if (lines && lines > 1) {
      return (
        <div ref={ref} className={cn('space-y-2', className)} {...props}>
          {Array.from({ length: lines }).map((_, i) => (
            <div
              key={i}
              className={cn(
                baseClasses,
                i === lines - 1 && 'w-3/4' // Last line is shorter
              )}
              style={{
                width: i === lines - 1 ? '75%' : width,
                height: height || '1rem',
                ...style,
              }}
            />
          ))}
        </div>
      );
    }

    return (
      <div
        ref={ref}
        className={cn(baseClasses, className)}
        style={{
          width,
          height,
          ...style,
        }}
        {...props}
      />
    );
  }
);

ShimmerSkeleton.displayName = 'ShimmerSkeleton';

// Pre-composed skeleton variants for common use cases
export interface SkeletonCardProps extends HTMLAttributes<HTMLDivElement> {
  showAvatar?: boolean;
  lines?: number;
}

const SkeletonCard = forwardRef<HTMLDivElement, SkeletonCardProps>(
  ({ className, showAvatar = true, lines = 3, ...props }, ref) => (
    <div
      ref={ref}
      className={cn(
        'p-4 rounded-2xl',
        'bg-white/70 dark:bg-gray-900/70',
        'backdrop-blur-xl',
        'border border-white/30 dark:border-white/10',
        'shadow-glass dark:shadow-glass-dark',
        className
      )}
      {...props}
    >
      <div className="flex items-start gap-4">
        {showAvatar && (
          <ShimmerSkeleton
            variant="circular"
            width={40}
            height={40}
            className="flex-shrink-0"
          />
        )}
        <div className="flex-1 space-y-3">
          <ShimmerSkeleton height="1.25rem" width="60%" />
          <ShimmerSkeleton lines={lines} height="0.875rem" />
          <div className="flex gap-2 pt-2">
            <ShimmerSkeleton variant="rounded" width={80} height={24} />
            <ShimmerSkeleton variant="rounded" width={60} height={24} />
          </div>
        </div>
      </div>
    </div>
  )
);

SkeletonCard.displayName = 'SkeletonCard';

// Skeleton for task items
export interface SkeletonTaskProps extends HTMLAttributes<HTMLDivElement> {}

const SkeletonTask = forwardRef<HTMLDivElement, SkeletonTaskProps>(
  ({ className, ...props }, ref) => (
    <div
      ref={ref}
      className={cn(
        'p-4 rounded-2xl',
        'bg-white/70 dark:bg-gray-900/70',
        'backdrop-blur-xl',
        'border border-white/30 dark:border-white/10',
        'shadow-glass dark:shadow-glass-dark',
        className
      )}
      {...props}
    >
      <div className="flex items-start gap-3">
        <ShimmerSkeleton variant="rounded" width={20} height={20} className="mt-0.5" />
        <div className="flex-1 min-w-0 space-y-2">
          <ShimmerSkeleton height="1rem" width="70%" />
          <ShimmerSkeleton height="0.75rem" width="50%" />
          <div className="flex items-center gap-2 pt-1">
            <ShimmerSkeleton variant="rounded" width={72} height={22} />
            <ShimmerSkeleton height="0.75rem" width={80} />
          </div>
        </div>
        <div className="flex gap-1">
          <ShimmerSkeleton variant="rounded" width={32} height={32} />
          <ShimmerSkeleton variant="rounded" width={32} height={32} />
        </div>
      </div>
    </div>
  )
);

SkeletonTask.displayName = 'SkeletonTask';

// Skeleton for table rows
export interface SkeletonTableProps extends HTMLAttributes<HTMLDivElement> {
  rows?: number;
}

const SkeletonTable = forwardRef<HTMLDivElement, SkeletonTableProps>(
  ({ className, rows = 5, ...props }, ref) => (
    <div
      ref={ref}
      className={cn(
        'rounded-2xl overflow-hidden',
        'bg-white/70 dark:bg-gray-900/70',
        'backdrop-blur-xl',
        'border border-white/30 dark:border-white/10',
        'shadow-glass dark:shadow-glass-dark',
        className
      )}
      {...props}
    >
      {/* Header */}
      <div className="p-4 border-b border-white/20 dark:border-white/5 bg-white/50 dark:bg-gray-900/50">
        <div className="flex gap-4">
          <ShimmerSkeleton width={24} height={16} />
          <ShimmerSkeleton width={120} height={16} />
          <ShimmerSkeleton width={200} height={16} className="hidden md:block" />
          <ShimmerSkeleton width={80} height={16} />
          <ShimmerSkeleton width={100} height={16} className="hidden sm:block" />
          <ShimmerSkeleton width={32} height={16} />
        </div>
      </div>
      {/* Rows */}
      {Array.from({ length: rows }).map((_, i) => (
        <div
          key={i}
          className={cn(
            'p-4 flex gap-4 items-center',
            i !== rows - 1 && 'border-b border-white/10 dark:border-white/5'
          )}
        >
          <ShimmerSkeleton variant="rounded" width={20} height={20} />
          <ShimmerSkeleton width={120} height={16} />
          <ShimmerSkeleton width={180} height={16} className="hidden md:block" />
          <ShimmerSkeleton variant="rounded" width={80} height={24} />
          <ShimmerSkeleton width={90} height={16} className="hidden sm:block" />
          <ShimmerSkeleton variant="rounded" width={32} height={32} />
        </div>
      ))}
    </div>
  )
);

SkeletonTable.displayName = 'SkeletonTable';

export { ShimmerSkeleton, SkeletonCard, SkeletonTask, SkeletonTable };
