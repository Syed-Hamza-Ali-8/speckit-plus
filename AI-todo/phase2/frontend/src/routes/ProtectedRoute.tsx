import type { ReactNode } from 'react';
import { Navigate, useLocation } from 'react-router-dom';
import { useAuth } from '@/hooks/useAuth';
import { ROUTES } from '@/routes';

/**
 * Protected route component props
 */
export interface ProtectedRouteProps {
  children: ReactNode;
  redirectTo?: string;
}

/**
 * ProtectedRoute component
 * Wraps routes that require authentication
 * Redirects to login page if user is not authenticated
 * Captures current path and passes as returnTo query param
 *
 * Flow:
 * - Unauthenticated visit to /tasks
 * - Redirects to /login?returnTo=/tasks
 * - After login, user returns to /tasks
 */
export function ProtectedRoute({
  children,
  redirectTo = ROUTES.LOGIN,
}: ProtectedRouteProps) {
  const { isAuthenticated } = useAuth();
  const location = useLocation();

  if (!isAuthenticated) {
    // Capture current path (including search params) for return URL
    const currentPath = location.pathname + location.search;

    // Build redirect URL with returnTo param
    const searchParams = new URLSearchParams();
    searchParams.set('returnTo', currentPath);

    const redirectUrl = `${redirectTo}?${searchParams.toString()}`;

    return <Navigate to={redirectUrl} replace />;
  }

  return <>{children}</>;
}
