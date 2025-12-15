import { forwardRef, type HTMLAttributes } from 'react';
import { motion, type HTMLMotionProps } from 'framer-motion';
import { cn } from '@/lib/utils';

export interface GlassCardProps
  extends Omit<HTMLAttributes<HTMLDivElement>, 'onAnimationStart' | 'onDrag' | 'onDragEnd' | 'onDragStart'> {
  variant?: 'default' | 'elevated' | 'bordered' | 'gradient';
  hover?: boolean;
  glow?: boolean;
  blur?: 'sm' | 'md' | 'lg' | 'xl';
  padding?: 'none' | 'sm' | 'md' | 'lg';
}

const blurClasses = {
  sm: 'backdrop-blur-sm',
  md: 'backdrop-blur-md',
  lg: 'backdrop-blur-lg',
  xl: 'backdrop-blur-xl',
};

const paddingClasses = {
  none: '',
  sm: 'p-3',
  md: 'p-4',
  lg: 'p-6',
};

const variantClasses = {
  default: `
    bg-white/70 dark:bg-gray-900/70
    border border-white/30 dark:border-white/10
    shadow-glass dark:shadow-glass-dark
    text-gray-900 dark:text-gray-100
  `,
  elevated: `
    bg-white/80 dark:bg-gray-900/80
    border border-white/40 dark:border-white/15
    shadow-lg shadow-black/5 dark:shadow-black/20
    text-gray-900 dark:text-gray-100
  `,
  bordered: `
    bg-white/60 dark:bg-gray-900/60
    border-2 border-purple-200/50 dark:border-purple-800/50
    shadow-glass dark:shadow-glass-dark
    text-gray-900 dark:text-gray-100
  `,
  gradient: `
    bg-gradient-to-br from-white/80 via-white/60 to-purple-50/50
    dark:from-gray-900/80 dark:via-gray-900/60 dark:to-purple-950/50
    border border-white/30 dark:border-white/10
    shadow-glass dark:shadow-glass-dark
    text-gray-900 dark:text-gray-100
  `,
};

const GlassCard = forwardRef<HTMLDivElement, GlassCardProps>(
  (
    {
      className,
      variant = 'default',
      hover = true,
      glow = false,
      blur = 'xl',
      padding = 'md',
      children,
      ...props
    },
    ref
  ) => {
    const motionProps: HTMLMotionProps<'div'> = hover
      ? {
          whileHover: { y: -4, scale: 1.01 },
          transition: { type: 'spring', stiffness: 300, damping: 20 },
        }
      : {};

    return (
      <motion.div
        ref={ref}
        className={cn(
          'relative rounded-2xl overflow-hidden',
          'transition-all duration-300 ease-premium',
          blurClasses[blur],
          paddingClasses[padding],
          variantClasses[variant],
          hover && 'hover:shadow-xl hover:shadow-black/10 dark:hover:shadow-black/30',
          hover && 'hover:border-purple-300/50 dark:hover:border-purple-500/30',
          glow && 'shadow-glow',
          className
        )}
        {...motionProps}
        {...(props as HTMLMotionProps<'div'>)}
      >
        {/* Subtle gradient overlay */}
        <div className="absolute inset-0 bg-gradient-to-br from-white/5 to-transparent pointer-events-none" />

        {/* Content */}
        <div className="relative z-10">{children}</div>
      </motion.div>
    );
  }
);

GlassCard.displayName = 'GlassCard';

// GlassCardHeader component
export interface GlassCardHeaderProps extends HTMLAttributes<HTMLDivElement> {}

const GlassCardHeader = forwardRef<HTMLDivElement, GlassCardHeaderProps>(
  ({ className, ...props }, ref) => (
    <div
      ref={ref}
      className={cn('flex flex-col space-y-1.5 pb-4', className)}
      {...props}
    />
  )
);
GlassCardHeader.displayName = 'GlassCardHeader';

// GlassCardTitle component
export interface GlassCardTitleProps extends HTMLAttributes<HTMLHeadingElement> {}

const GlassCardTitle = forwardRef<HTMLHeadingElement, GlassCardTitleProps>(
  ({ className, ...props }, ref) => (
    <h3
      ref={ref}
      className={cn(
        'text-xl font-semibold leading-none tracking-tight',
        'text-foreground',
        className
      )}
      {...props}
    />
  )
);
GlassCardTitle.displayName = 'GlassCardTitle';

// GlassCardDescription component
export interface GlassCardDescriptionProps extends HTMLAttributes<HTMLParagraphElement> {}

const GlassCardDescription = forwardRef<HTMLParagraphElement, GlassCardDescriptionProps>(
  ({ className, ...props }, ref) => (
    <p
      ref={ref}
      className={cn('text-sm text-muted-foreground', className)}
      {...props}
    />
  )
);
GlassCardDescription.displayName = 'GlassCardDescription';

// GlassCardContent component
export interface GlassCardContentProps extends HTMLAttributes<HTMLDivElement> {}

const GlassCardContent = forwardRef<HTMLDivElement, GlassCardContentProps>(
  ({ className, ...props }, ref) => (
    <div ref={ref} className={cn('', className)} {...props} />
  )
);
GlassCardContent.displayName = 'GlassCardContent';

// GlassCardFooter component
export interface GlassCardFooterProps extends HTMLAttributes<HTMLDivElement> {}

const GlassCardFooter = forwardRef<HTMLDivElement, GlassCardFooterProps>(
  ({ className, ...props }, ref) => (
    <div
      ref={ref}
      className={cn('flex items-center pt-4', className)}
      {...props}
    />
  )
);
GlassCardFooter.displayName = 'GlassCardFooter';

export {
  GlassCard,
  GlassCardHeader,
  GlassCardTitle,
  GlassCardDescription,
  GlassCardContent,
  GlassCardFooter,
};
