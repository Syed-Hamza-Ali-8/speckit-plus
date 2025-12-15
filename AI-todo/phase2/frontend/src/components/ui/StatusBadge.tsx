import { forwardRef, type HTMLAttributes } from 'react';
import { motion } from 'framer-motion';
import {
  CheckCircle2,
  Clock,
  AlertCircle,
  XCircle,
  Loader2,
  Sparkles,
  type LucideIcon,
} from 'lucide-react';
import { cn } from '@/lib/utils';

export type StatusType =
  | 'pending'
  | 'completed'
  | 'in_progress'
  | 'cancelled'
  | 'error'
  | 'success'
  | 'warning'
  | 'info';

export interface StatusBadgeProps {
  status: StatusType;
  size?: 'sm' | 'md' | 'lg';
  showIcon?: boolean;
  pulse?: boolean;
  label?: string;
  className?: string;
}

interface StatusConfig {
  icon: LucideIcon;
  label: string;
  classes: string;
  iconClasses: string;
  gradient?: string;
}

const statusConfig: Record<StatusType, StatusConfig> = {
  pending: {
    icon: Clock,
    label: 'Pending',
    classes:
      'bg-amber-500/10 text-amber-600 dark:text-amber-400 border-amber-500/20 dark:border-amber-400/20',
    iconClasses: 'text-amber-500 dark:text-amber-400',
    gradient: 'from-amber-500/20 to-orange-500/20',
  },
  completed: {
    icon: CheckCircle2,
    label: 'Completed',
    classes:
      'bg-emerald-500/10 text-emerald-600 dark:text-emerald-400 border-emerald-500/20 dark:border-emerald-400/20',
    iconClasses: 'text-emerald-500 dark:text-emerald-400',
    gradient: 'from-emerald-500/20 to-green-500/20',
  },
  in_progress: {
    icon: Loader2,
    label: 'In Progress',
    classes:
      'bg-blue-500/10 text-blue-600 dark:text-blue-400 border-blue-500/20 dark:border-blue-400/20',
    iconClasses: 'text-blue-500 dark:text-blue-400 animate-spin',
    gradient: 'from-blue-500/20 to-cyan-500/20',
  },
  cancelled: {
    icon: XCircle,
    label: 'Cancelled',
    classes:
      'bg-gray-500/10 text-gray-600 dark:text-gray-400 border-gray-500/20 dark:border-gray-400/20',
    iconClasses: 'text-gray-500 dark:text-gray-400',
    gradient: 'from-gray-500/20 to-slate-500/20',
  },
  error: {
    icon: XCircle,
    label: 'Error',
    classes:
      'bg-red-500/10 text-red-600 dark:text-red-400 border-red-500/20 dark:border-red-400/20',
    iconClasses: 'text-red-500 dark:text-red-400',
    gradient: 'from-red-500/20 to-rose-500/20',
  },
  success: {
    icon: Sparkles,
    label: 'Success',
    classes:
      'bg-emerald-500/10 text-emerald-600 dark:text-emerald-400 border-emerald-500/20 dark:border-emerald-400/20',
    iconClasses: 'text-emerald-500 dark:text-emerald-400',
    gradient: 'from-emerald-500/20 to-teal-500/20',
  },
  warning: {
    icon: AlertCircle,
    label: 'Warning',
    classes:
      'bg-amber-500/10 text-amber-600 dark:text-amber-400 border-amber-500/20 dark:border-amber-400/20',
    iconClasses: 'text-amber-500 dark:text-amber-400',
    gradient: 'from-amber-500/20 to-yellow-500/20',
  },
  info: {
    icon: AlertCircle,
    label: 'Info',
    classes:
      'bg-blue-500/10 text-blue-600 dark:text-blue-400 border-blue-500/20 dark:border-blue-400/20',
    iconClasses: 'text-blue-500 dark:text-blue-400',
    gradient: 'from-blue-500/20 to-indigo-500/20',
  },
};

const sizeClasses = {
  sm: 'text-xs px-2 py-0.5',
  md: 'text-xs px-2.5 py-1',
  lg: 'text-sm px-3 py-1.5',
};

const iconSizeClasses = {
  sm: 'h-3 w-3',
  md: 'h-3.5 w-3.5',
  lg: 'h-4 w-4',
};

const StatusBadge = forwardRef<HTMLSpanElement, StatusBadgeProps>(
  (
    {
      status,
      size = 'md',
      showIcon = true,
      pulse = false,
      label,
      className,
    },
    ref
  ) => {
    const config = statusConfig[status];
    const Icon = config.icon;
    const displayLabel = label || config.label;

    return (
      <motion.span
        ref={ref}
        initial={{ scale: 0.9, opacity: 0 }}
        animate={{ scale: 1, opacity: 1 }}
        transition={{ type: 'spring', stiffness: 500, damping: 30 }}
        className={cn(
          'inline-flex items-center gap-1.5',
          'font-medium rounded-full',
          'border backdrop-blur-sm',
          'transition-all duration-200',
          'relative',
          sizeClasses[size],
          config.classes,
          pulse && 'animate-pulse',
          className
        )}
      >
        {/* Gradient background overlay */}
        <span
          className={cn(
            'absolute inset-0 rounded-full bg-gradient-to-r opacity-50',
            config.gradient
          )}
        />

        {/* Icon */}
        {showIcon && (
          <Icon
            className={cn(
              'relative z-10 flex-shrink-0',
              iconSizeClasses[size],
              config.iconClasses
            )}
          />
        )}

        {/* Label */}
        <span className="relative z-10">{displayLabel}</span>
      </motion.span>
    );
  }
);

StatusBadge.displayName = 'StatusBadge';

// Priority badge variant
export type PriorityType = 'low' | 'medium' | 'high' | 'urgent';

export interface PriorityBadgeProps extends HTMLAttributes<HTMLSpanElement> {
  priority: PriorityType;
  size?: 'sm' | 'md' | 'lg';
}

const priorityConfig: Record<PriorityType, { label: string; classes: string }> = {
  low: {
    label: 'Low',
    classes:
      'bg-slate-500/10 text-slate-600 dark:text-slate-400 border-slate-500/20',
  },
  medium: {
    label: 'Medium',
    classes:
      'bg-blue-500/10 text-blue-600 dark:text-blue-400 border-blue-500/20',
  },
  high: {
    label: 'High',
    classes:
      'bg-orange-500/10 text-orange-600 dark:text-orange-400 border-orange-500/20',
  },
  urgent: {
    label: 'Urgent',
    classes:
      'bg-red-500/10 text-red-600 dark:text-red-400 border-red-500/20 animate-pulse',
  },
};

const PriorityBadge = forwardRef<HTMLSpanElement, PriorityBadgeProps>(
  ({ priority, size = 'md', className }, ref) => {
    const config = priorityConfig[priority];

    return (
      <motion.span
        ref={ref}
        initial={{ scale: 0.9, opacity: 0 }}
        animate={{ scale: 1, opacity: 1 }}
        transition={{ type: 'spring', stiffness: 500, damping: 30 }}
        className={cn(
          'inline-flex items-center gap-1',
          'font-medium rounded-full',
          'border backdrop-blur-sm',
          sizeClasses[size],
          config.classes,
          className
        )}
      >
        <span
          className={cn(
            'w-1.5 h-1.5 rounded-full',
            priority === 'low' && 'bg-slate-500',
            priority === 'medium' && 'bg-blue-500',
            priority === 'high' && 'bg-orange-500',
            priority === 'urgent' && 'bg-red-500 animate-ping'
          )}
        />
        {config.label}
      </motion.span>
    );
  }
);

PriorityBadge.displayName = 'PriorityBadge';

export { StatusBadge, PriorityBadge };
