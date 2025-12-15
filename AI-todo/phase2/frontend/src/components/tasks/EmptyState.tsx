import { motion } from 'framer-motion';
import { ClipboardList, Plus, X, Sparkles } from 'lucide-react';
import { GlassButton } from '@/components/ui/GlassButton';
import { GlassCard } from '@/components/ui/GlassCard';
import { cn } from '@/lib/utils';

export interface EmptyStateProps {
  hasFilters: boolean;
  onCreateClick: () => void;
  onClearFilters: () => void;
}

export function EmptyState({ hasFilters, onCreateClick, onClearFilters }: EmptyStateProps) {
  return (
    <GlassCard variant="gradient" hover={false} padding="lg" className="max-w-md mx-auto">
      <div className="flex flex-col items-center justify-center py-8 px-4 text-center">
        {/* Animated icon container */}
        <motion.div
          initial={{ scale: 0.8, opacity: 0 }}
          animate={{ scale: 1, opacity: 1 }}
          transition={{ type: 'spring', stiffness: 200, damping: 15 }}
          className="relative mb-6"
        >
          {/* Background glow */}
          <div
            className={cn(
              'absolute inset-0 rounded-full blur-xl',
              'bg-gradient-to-br from-purple-500/30 via-blue-500/30 to-indigo-500/30'
            )}
          />

          {/* Icon container */}
          <motion.div
            animate={{ y: [0, -5, 0] }}
            transition={{ repeat: Infinity, duration: 2, ease: 'easeInOut' }}
            className={cn(
              'relative rounded-2xl p-5',
              'bg-gradient-to-br from-purple-500 via-blue-500 to-indigo-600',
              'shadow-premium'
            )}
          >
            <ClipboardList className="h-10 w-10 text-white" />

            {/* Sparkle decorations */}
            <motion.div
              animate={{ rotate: 360 }}
              transition={{ repeat: Infinity, duration: 8, ease: 'linear' }}
              className="absolute -top-2 -right-2"
            >
              <Sparkles className="h-4 w-4 text-amber-400" />
            </motion.div>
          </motion.div>
        </motion.div>

        {hasFilters ? (
          // Filtered empty state
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.2 }}
          >
            <h3 className="text-xl font-bold mb-2 text-foreground">
              No matching tasks
            </h3>
            <p className="text-sm text-muted-foreground mb-6 max-w-xs">
              We couldn't find any tasks matching your current filters. Try adjusting your search criteria.
            </p>
            <GlassButton
              variant="ghost"
              onClick={onClearFilters}
              leftIcon={<X className="h-4 w-4" />}
            >
              Clear filters
            </GlassButton>
          </motion.div>
        ) : (
          // Default empty state
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.2 }}
          >
            <h3 className="text-xl font-bold mb-2 text-foreground">
              Ready to get productive?
            </h3>
            <p className="text-sm text-muted-foreground mb-6 max-w-xs">
              Create your first task and start organizing your work like a pro.
            </p>
            <GlassButton
              variant="premium"
              glow
              onClick={onCreateClick}
              leftIcon={<Plus className="h-4 w-4" />}
            >
              Create your first task
            </GlassButton>
          </motion.div>
        )}
      </div>
    </GlassCard>
  );
}

export default EmptyState;
