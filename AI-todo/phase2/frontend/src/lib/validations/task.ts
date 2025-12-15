import { z } from 'zod';

/**
 * Task form validation schema
 * - Title: required, max 100 characters
 * - Description: optional, max 500 characters
 * - Due date: optional, must be today or future
 */
export const taskSchema = z.object({
  title: z
    .string()
    .min(1, 'Title is required')
    .max(100, 'Title must be at most 100 characters'),
  description: z
    .string()
    .max(500, 'Description must be at most 500 characters')
    .optional()
    .or(z.literal('')),
  dueDate: z
    .string()
    .optional()
    .or(z.literal('')),
});

/**
 * Task edit form validation schema (includes status)
 * - Title: required, max 100 characters
 * - Description: optional, max 500 characters
 * - Status: pending or completed
 * - Due date: optional
 */
export const taskEditSchema = z.object({
  title: z
    .string()
    .min(1, 'Title is required')
    .max(100, 'Title must be at most 100 characters'),
  description: z
    .string()
    .max(500, 'Description must be at most 500 characters')
    .optional()
    .or(z.literal('')),
  status: z.enum(['pending', 'completed']).optional(),
  dueDate: z
    .string()
    .optional()
    .or(z.literal('')),
});

// TypeScript types inferred from schemas
export type TaskFormData = z.infer<typeof taskSchema>;
export type TaskEditFormData = z.infer<typeof taskEditSchema>;
