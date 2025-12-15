import { z } from 'zod';

/**
 * Login form validation schema
 * - Email: required, valid email format
 * - Password: required, minimum 8 characters
 */
export const loginSchema = z.object({
  email: z
    .string()
    .min(1, 'Email is required')
    .email('Invalid email address'),
  password: z
    .string()
    .min(1, 'Password is required')
    .min(8, 'Password must be at least 8 characters'),
});

/**
 * Registration form validation schema
 * - First Name: optional, max 100 characters
 * - Last Name: optional, max 100 characters
 * - Email: required, valid email format
 * - Password: required, minimum 8 characters
 * - Confirm Password: must match password field
 */
export const registerSchema = z
  .object({
    firstName: z
      .string()
      .max(100, 'First name must be less than 100 characters')
      .optional(),
    lastName: z
      .string()
      .max(100, 'Last name must be less than 100 characters')
      .optional(),
    email: z
      .string()
      .min(1, 'Email is required')
      .email('Invalid email address'),
    password: z
      .string()
      .min(1, 'Password is required')
      .min(8, 'Password must be at least 8 characters'),
    confirmPassword: z
      .string()
      .min(1, 'Please confirm your password'),
  })
  .refine((data) => data.password === data.confirmPassword, {
    message: 'Passwords do not match',
    path: ['confirmPassword'],
  });

// TypeScript types inferred from schemas
export type LoginFormData = z.infer<typeof loginSchema>;
export type RegisterFormData = z.infer<typeof registerSchema>;
