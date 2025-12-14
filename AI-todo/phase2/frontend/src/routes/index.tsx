/**
 * Application route definitions
 */
export const ROUTES = {
  HOME: '/',
  LOGIN: '/login',
  REGISTER: '/register',
  TASKS: '/tasks',
} as const;

export type RoutePath = (typeof ROUTES)[keyof typeof ROUTES];
