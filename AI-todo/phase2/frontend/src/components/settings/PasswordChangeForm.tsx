import { useState } from 'react';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { motion } from 'framer-motion';
import { Eye, EyeOff, Lock, Check } from 'lucide-react';
import { toast } from 'sonner';
import { useChangePasswordMutation } from '@/services/settingsApi';
import {
  passwordChangeSchema,
  type PasswordChangeFormData,
} from '@/lib/validations/settings';
import { GlassButton } from '@/components/ui/GlassButton';
import { cn } from '@/lib/utils';

/**
 * Password change form component
 * Validates current password, new password, and confirmation
 */
export function PasswordChangeForm() {
  const [showCurrentPassword, setShowCurrentPassword] = useState(false);
  const [showNewPassword, setShowNewPassword] = useState(false);
  const [showConfirmPassword, setShowConfirmPassword] = useState(false);
  const [changePassword, { isLoading }] = useChangePasswordMutation();

  const {
    register,
    handleSubmit,
    reset,
    formState: { errors },
  } = useForm<PasswordChangeFormData>({
    resolver: zodResolver(passwordChangeSchema),
  });

  const onSubmit = async (data: PasswordChangeFormData) => {
    try {
      await changePassword({
        current_password: data.currentPassword,
        new_password: data.newPassword,
      }).unwrap();

      toast.success('Password changed successfully');
      reset();
    } catch (error: unknown) {
      const err = error as { data?: { detail?: string } };
      if (err.data?.detail === 'Current password is incorrect') {
        toast.error('Current password is incorrect');
      } else {
        toast.error('Failed to change password');
      }
    }
  };

  return (
    <form onSubmit={handleSubmit(onSubmit)} className="space-y-4">
      <p className="text-sm text-gray-600 dark:text-gray-400 mb-4">
        Update your password to keep your account secure
      </p>

      {/* Current Password */}
      <div className="space-y-2">
        <label className="text-sm font-medium text-gray-900 dark:text-gray-100">
          Current Password
        </label>
        <div className="relative">
          <div className="absolute left-3 top-1/2 -translate-y-1/2">
            <Lock className="h-4 w-4 text-gray-600 dark:text-gray-400" />
          </div>
          <input
            type={showCurrentPassword ? 'text' : 'password'}
            placeholder="Enter current password"
            {...register('currentPassword')}
            className={cn(
              'w-full h-11 pl-10 pr-10 rounded-xl',
              'bg-white/70 dark:bg-gray-800/70',
              'border border-white/30 dark:border-white/10',
              'text-gray-900 dark:text-gray-100 placeholder:text-gray-500 dark:placeholder:text-gray-400',
              'focus:outline-none focus:ring-2 focus:ring-purple-500/50',
              errors.currentPassword && 'border-red-500/50 focus:ring-red-500/50'
            )}
          />
          <button
            type="button"
            onClick={() => setShowCurrentPassword(!showCurrentPassword)}
            className="absolute right-3 top-1/2 -translate-y-1/2 text-gray-600 dark:text-gray-400 hover:text-gray-900 dark:hover:text-gray-100"
          >
            {showCurrentPassword ? (
              <EyeOff className="h-4 w-4" />
            ) : (
              <Eye className="h-4 w-4" />
            )}
          </button>
        </div>
        {errors.currentPassword && (
          <p className="text-sm text-red-500">{errors.currentPassword.message}</p>
        )}
      </div>

      {/* New Password */}
      <div className="space-y-2">
        <label className="text-sm font-medium text-gray-900 dark:text-gray-100">
          New Password
        </label>
        <div className="relative">
          <div className="absolute left-3 top-1/2 -translate-y-1/2">
            <Lock className="h-4 w-4 text-gray-600 dark:text-gray-400" />
          </div>
          <input
            type={showNewPassword ? 'text' : 'password'}
            placeholder="Enter new password (min 8 characters)"
            {...register('newPassword')}
            className={cn(
              'w-full h-11 pl-10 pr-10 rounded-xl',
              'bg-white/70 dark:bg-gray-800/70',
              'border border-white/30 dark:border-white/10',
              'text-gray-900 dark:text-gray-100 placeholder:text-gray-500 dark:placeholder:text-gray-400',
              'focus:outline-none focus:ring-2 focus:ring-purple-500/50',
              errors.newPassword && 'border-red-500/50 focus:ring-red-500/50'
            )}
          />
          <button
            type="button"
            onClick={() => setShowNewPassword(!showNewPassword)}
            className="absolute right-3 top-1/2 -translate-y-1/2 text-gray-600 dark:text-gray-400 hover:text-gray-900 dark:hover:text-gray-100"
          >
            {showNewPassword ? (
              <EyeOff className="h-4 w-4" />
            ) : (
              <Eye className="h-4 w-4" />
            )}
          </button>
        </div>
        {errors.newPassword && (
          <p className="text-sm text-red-500">{errors.newPassword.message}</p>
        )}
      </div>

      {/* Confirm Password */}
      <div className="space-y-2">
        <label className="text-sm font-medium text-gray-900 dark:text-gray-100">
          Confirm New Password
        </label>
        <div className="relative">
          <div className="absolute left-3 top-1/2 -translate-y-1/2">
            <Check className="h-4 w-4 text-gray-600 dark:text-gray-400" />
          </div>
          <input
            type={showConfirmPassword ? 'text' : 'password'}
            placeholder="Confirm new password"
            {...register('confirmPassword')}
            className={cn(
              'w-full h-11 pl-10 pr-10 rounded-xl',
              'bg-white/70 dark:bg-gray-800/70',
              'border border-white/30 dark:border-white/10',
              'text-gray-900 dark:text-gray-100 placeholder:text-gray-500 dark:placeholder:text-gray-400',
              'focus:outline-none focus:ring-2 focus:ring-purple-500/50',
              errors.confirmPassword && 'border-red-500/50 focus:ring-red-500/50'
            )}
          />
          <button
            type="button"
            onClick={() => setShowConfirmPassword(!showConfirmPassword)}
            className="absolute right-3 top-1/2 -translate-y-1/2 text-gray-600 dark:text-gray-400 hover:text-gray-900 dark:hover:text-gray-100"
          >
            {showConfirmPassword ? (
              <EyeOff className="h-4 w-4" />
            ) : (
              <Eye className="h-4 w-4" />
            )}
          </button>
        </div>
        {errors.confirmPassword && (
          <p className="text-sm text-red-500">{errors.confirmPassword.message}</p>
        )}
      </div>

      {/* Submit button */}
      <motion.div whileHover={{ scale: 1.01 }} whileTap={{ scale: 0.99 }}>
        <GlassButton
          type="submit"
          variant="premium"
          className="w-full"
          disabled={isLoading}
        >
          {isLoading ? 'Changing Password...' : 'Change Password'}
        </GlassButton>
      </motion.div>
    </form>
  );
}
