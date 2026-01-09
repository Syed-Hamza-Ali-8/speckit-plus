import { z } from 'zod';

/**
 * Profile form validation schema
 * - First Name: optional, max 100 characters
 * - Last Name: optional, max 100 characters
 */
export const profileSchema = z.object({
  firstName: z
    .string()
    .max(100, 'First name must be less than 100 characters')
    .nullable()
    .optional(),
  lastName: z
    .string()
    .max(100, 'Last name must be less than 100 characters')
    .nullable()
    .optional(),
});

// TypeScript type inferred from schema
export type ProfileFormData = z.infer<typeof profileSchema>;
