/**
 * Application route definitions
 */
export const ROUTES = {
  HOME: '/',
  LOGIN: '/login',
  REGISTER: '/register',
  TASKS: '/tasks',
  RECURRING_PATTERNS: '/recurring-patterns',
  PROFILE: '/profile',
  SETTINGS: '/settings',
  CHAT: '/chat',
} as const;

export type RoutePath = (typeof ROUTES)[keyof typeof ROUTES];
