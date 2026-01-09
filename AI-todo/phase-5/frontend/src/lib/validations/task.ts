import { z } from 'zod';

/**
 * Task form validation schema
 * - Title: required, max 100 characters
 * - Description: optional, max 500 characters
 * - Due date: optional, must be today or future
 * - Priority: optional, low/medium/high
 * - Tags: optional, array of strings
 * - Is recurring: optional, boolean
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
  priority: z.enum(['low', 'medium', 'high']).optional(),
  tags: z.array(z.string()).optional(),
  isRecurring: z.boolean().optional(),
});

/**
 * Task edit form validation schema (includes status)
 * - Title: required, max 100 characters
 * - Description: optional, max 500 characters
 * - Status: pending or completed
 * - Due date: optional
 * - Priority: optional, low/medium/high
 * - Tags: optional, array of strings
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
  priority: z.enum(['low', 'medium', 'high']).optional(),
  tags: z.array(z.string()).optional(),
});

// Recurring task pattern validation schema
export const recurringTaskPatternSchema = z.object({
  base_task_title: z
    .string()
    .min(1, 'Base task title is required')
    .max(100, 'Base task title must be at most 100 characters'),
  base_task_description: z
    .string()
    .max(500, 'Description must be at most 500 characters')
    .optional()
    .or(z.literal('')),
  pattern_type: z.enum(['daily', 'weekly', 'monthly', 'yearly']),
  interval: z
    .number()
    .min(1, 'Interval must be at least 1')
    .max(365, 'Interval must be at most 365'),
  start_date: z
    .string()
    .min(1, 'Start date is required')
    .regex(/^\d{4}-\d{2}-\d{2}$/, 'Date must be in YYYY-MM-DD format'),
  end_date: z
    .string()
    .regex(/^\d{4}-\d{2}-\d{2}$/, 'Date must be in YYYY-MM-DD format')
    .optional()
    .or(z.literal('')),
  weekdays: z.array(z.number()).optional(),
  days_of_month: z.array(z.number()).optional(),
});

// TypeScript types inferred from schemas
export type TaskFormData = z.infer<typeof taskSchema>;
export type TaskEditFormData = z.infer<typeof taskEditSchema>;
export type RecurringTaskPatternFormData = z.infer<typeof recurringTaskPatternSchema>;
