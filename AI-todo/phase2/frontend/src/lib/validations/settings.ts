import { z } from 'zod';

/**
 * Password change form validation schema
 * Validates current password, new password (min 8 chars), and confirmation match
 */
export const passwordChangeSchema = z
  .object({
    currentPassword: z.string().min(1, 'Current password is required'),
    newPassword: z
      .string()
      .min(8, 'Password must be at least 8 characters'),
    confirmPassword: z.string().min(1, 'Please confirm your password'),
  })
  .refine((data) => data.newPassword === data.confirmPassword, {
    message: 'Passwords do not match',
    path: ['confirmPassword'],
  });

/**
 * Inferred type from password change schema
 */
export type PasswordChangeFormData = z.infer<typeof passwordChangeSchema>;
