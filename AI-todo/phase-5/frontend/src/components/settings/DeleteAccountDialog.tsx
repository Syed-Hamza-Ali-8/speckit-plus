import { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { motion, AnimatePresence } from 'framer-motion';
import { AlertTriangle, Trash2, X } from 'lucide-react';
import { toast } from 'sonner';
import { useDeleteAccountMutation } from '@/services/settingsApi';
import { useAuth } from '@/hooks/useAuth';
import { GlassButton } from '@/components/ui/GlassButton';
import { ROUTES } from '@/routes';
import { cn } from '@/lib/utils';

/**
 * Delete account dialog component
 * Requires typing "DELETE" to confirm account deletion
 */
export function DeleteAccountDialog() {
  const [isOpen, setIsOpen] = useState(false);
  const [confirmText, setConfirmText] = useState('');
  const [deleteAccount, { isLoading }] = useDeleteAccountMutation();
  const { logout } = useAuth();
  const navigate = useNavigate();

  const isConfirmed = confirmText === 'DELETE';

  const handleDelete = async () => {
    if (!isConfirmed) return;

    try {
      await deleteAccount().unwrap();
      toast.success('Account deleted successfully');
      logout();
      navigate(ROUTES.HOME);
    } catch {
      toast.error('Failed to delete account');
    }
  };

  const handleClose = () => {
    setIsOpen(false);
    setConfirmText('');
  };

  return (
    <>
      {/* Trigger section */}
      <div className="space-y-3">
        <p className="text-sm text-gray-600 dark:text-gray-400">
          Once you delete your account, there is no going back. Please be certain.
        </p>
        <motion.div whileHover={{ scale: 1.01 }} whileTap={{ scale: 0.99 }}>
          <button
            onClick={() => setIsOpen(true)}
            className={cn(
              'flex items-center gap-2 px-4 py-2 rounded-xl',
              'bg-red-500/10 hover:bg-red-500/20',
              'border border-red-500/30',
              'text-red-600 dark:text-red-400 font-medium',
              'transition-colors duration-200'
            )}
          >
            <Trash2 className="h-4 w-4" />
            Delete Account
          </button>
        </motion.div>
      </div>

      {/* Dialog */}
      <AnimatePresence>
        {isOpen && (
          <>
            {/* Backdrop */}
            <motion.div
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              exit={{ opacity: 0 }}
              onClick={handleClose}
              className="fixed inset-0 z-50 bg-black/50 backdrop-blur-sm"
            />

            {/* Dialog content */}
            <motion.div
              initial={{ opacity: 0, scale: 0.95, y: 20 }}
              animate={{ opacity: 1, scale: 1, y: 0 }}
              exit={{ opacity: 0, scale: 0.95, y: 20 }}
              transition={{ type: 'spring', stiffness: 300, damping: 25 }}
              className={cn(
                'fixed z-50 left-1/2 top-1/2 -translate-x-1/2 -translate-y-1/2',
                'w-full max-w-md p-6 rounded-2xl',
                'bg-white/95 dark:bg-gray-900/95',
                'backdrop-blur-xl',
                'border border-white/30 dark:border-white/10',
                'shadow-2xl shadow-black/20'
              )}
            >
              {/* Close button */}
              <button
                onClick={handleClose}
                className={cn(
                  'absolute top-4 right-4 p-1.5 rounded-lg',
                  'text-gray-600 dark:text-gray-400 hover:text-gray-900 dark:hover:text-gray-100',
                  'hover:bg-gray-100 dark:hover:bg-gray-800',
                  'transition-colors'
                )}
              >
                <X className="h-5 w-5" />
              </button>

              {/* Warning icon */}
              <div className="flex justify-center mb-4">
                <div
                  className={cn(
                    'flex items-center justify-center',
                    'w-16 h-16 rounded-full',
                    'bg-red-100 dark:bg-red-900/30'
                  )}
                >
                  <AlertTriangle className="h-8 w-8 text-red-600 dark:text-red-400" />
                </div>
              </div>

              {/* Title */}
              <h2 className="text-xl font-bold text-center text-gray-900 dark:text-gray-100 mb-2">
                Delete Account
              </h2>

              {/* Description */}
              <p className="text-center text-gray-600 dark:text-gray-400 mb-6">
                This action cannot be undone. This will permanently delete your account
                and remove all your data including tasks.
              </p>

              {/* Confirmation input */}
              <div className="space-y-2 mb-6">
                <label className="text-sm font-medium text-gray-900 dark:text-gray-100">
                  Type <span className="font-bold text-red-600">DELETE</span> to confirm
                </label>
                <input
                  type="text"
                  value={confirmText}
                  onChange={(e) => setConfirmText(e.target.value)}
                  placeholder="Type DELETE"
                  className={cn(
                    'w-full h-11 px-4 rounded-xl',
                    'bg-white/70 dark:bg-gray-800/70',
                    'border border-white/30 dark:border-white/10',
                    'text-gray-900 dark:text-gray-100 placeholder:text-gray-500 dark:placeholder:text-gray-400',
                    'focus:outline-none focus:ring-2',
                    isConfirmed
                      ? 'focus:ring-red-500/50 border-red-500/50'
                      : 'focus:ring-purple-500/50'
                  )}
                />
              </div>

              {/* Actions */}
              <div className="flex gap-3">
                <GlassButton
                  variant="ghost"
                  className="flex-1"
                  onClick={handleClose}
                >
                  Cancel
                </GlassButton>
                <motion.button
                  whileHover={isConfirmed ? { scale: 1.02 } : {}}
                  whileTap={isConfirmed ? { scale: 0.98 } : {}}
                  onClick={handleDelete}
                  disabled={!isConfirmed || isLoading}
                  className={cn(
                    'flex-1 flex items-center justify-center gap-2',
                    'px-4 py-2.5 rounded-xl font-medium',
                    'transition-all duration-200',
                    isConfirmed
                      ? 'bg-red-600 hover:bg-red-700 text-white cursor-pointer'
                      : 'bg-gray-200 dark:bg-gray-700 text-gray-400 cursor-not-allowed'
                  )}
                >
                  {isLoading ? (
                    'Deleting...'
                  ) : (
                    <>
                      <Trash2 className="h-4 w-4" />
                      Delete Account
                    </>
                  )}
                </motion.button>
              </div>
            </motion.div>
          </>
        )}
      </AnimatePresence>
    </>
  );
}
