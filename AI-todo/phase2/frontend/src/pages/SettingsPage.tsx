import { motion } from 'framer-motion';
import { Settings } from 'lucide-react';
import { ThemeSettings } from '@/components/settings/ThemeSettings';
import { NotificationSettings } from '@/components/settings/NotificationSettings';
import { PasswordChangeForm } from '@/components/settings/PasswordChangeForm';
import { DeleteAccountDialog } from '@/components/settings/DeleteAccountDialog';
import { GlassCard } from '@/components/ui/GlassCard';
import { cn } from '@/lib/utils';

/**
 * Settings page
 * Allows users to manage theme, notifications, password, and account deletion
 */
export function SettingsPage() {
  return (
    <motion.div
      initial={{ opacity: 0, y: 20 }}
      animate={{ opacity: 1, y: 0 }}
      transition={{ duration: 0.3 }}
      className="container mx-auto px-4 py-8 max-w-3xl"
    >
      {/* Page header */}
      <div className="mb-8">
        <div className="flex items-center gap-3 mb-2">
          <div
            className={cn(
              'flex items-center justify-center',
              'w-10 h-10 rounded-xl',
              'bg-gradient-to-br from-purple-500 via-blue-500 to-indigo-600',
              'shadow-premium'
            )}
          >
            <Settings className="h-5 w-5 text-white" />
          </div>
          <h1 className="text-2xl font-bold text-gray-900 dark:text-gray-100">Settings</h1>
        </div>
        <p className="text-gray-600 dark:text-gray-400">
          Manage your account settings and preferences
        </p>
      </div>

      {/* Settings sections */}
      <div className="space-y-6">
        {/* Appearance section */}
        <GlassCard className="p-6">
          <h2 className="text-lg font-semibold text-gray-900 dark:text-gray-100 mb-4">
            Appearance
          </h2>
          <ThemeSettings />
        </GlassCard>

        {/* Notifications section */}
        <GlassCard className="p-6">
          <h2 className="text-lg font-semibold text-gray-900 dark:text-gray-100 mb-4">
            Notifications
          </h2>
          <NotificationSettings />
        </GlassCard>

        {/* Security section */}
        <GlassCard className="p-6">
          <h2 className="text-lg font-semibold text-gray-900 dark:text-gray-100 mb-4">
            Security
          </h2>
          <PasswordChangeForm />
        </GlassCard>

        {/* Danger zone */}
        <GlassCard className="p-6 border-red-500/20">
          <h2 className="text-lg font-semibold text-red-600 dark:text-red-400 mb-4">
            Danger Zone
          </h2>
          <DeleteAccountDialog />
        </GlassCard>
      </div>
    </motion.div>
  );
}
