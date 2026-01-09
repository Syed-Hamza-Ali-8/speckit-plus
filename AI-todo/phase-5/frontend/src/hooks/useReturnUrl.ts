import { useSearchParams } from 'react-router-dom';

/**
 * Hook for extracting and validating return URL from search params
 *
 * Features:
 * - Extracts `returnTo` from URL search params
 * - Validates that returnTo is an internal path (starts with /)
 * - Provides default fallback to /tasks
 * - Prevents open redirect vulnerabilities
 *
 * Usage:
 * ```tsx
 * const returnUrl = useReturnUrl();
 * // After successful login:
 * navigate(returnUrl);
 * ```
 */
export function useReturnUrl(defaultPath: string = '/tasks'): string {
  const [searchParams] = useSearchParams();
  const returnTo = searchParams.get('returnTo');

  // Validate returnTo is an internal path (must start with /)
  // Prevent open redirect by rejecting:
  // - External URLs (http://, https://, //)
  // - Protocol-relative URLs (//example.com)
  // - javascript: URLs
  if (returnTo) {
    const trimmed = returnTo.trim();

    // Must start with single forward slash (internal path)
    // Must not start with // (protocol-relative)
    // Must not contain protocol (://)
    if (
      trimmed.startsWith('/') &&
      !trimmed.startsWith('//') &&
      !trimmed.includes('://')
    ) {
      return trimmed;
    }
  }

  // Return default path if no valid returnTo
  return defaultPath;
}
