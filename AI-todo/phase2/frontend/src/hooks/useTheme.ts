import { useEffect } from 'react';
import { useUIStore } from '@/stores/uiStore';
import type { Theme } from '@/stores/uiStore';

/**
 * useTheme hook
 * Provides theme state and applies dark mode class to document.documentElement
 * Handles system preference detection and changes
 */
export function useTheme() {
  const theme = useUIStore((state) => state.theme);
  const setTheme = useUIStore((state) => state.setTheme);

  // Apply theme class to document.documentElement
  useEffect(() => {
    const root = document.documentElement;

    const applyTheme = (resolvedTheme: 'light' | 'dark') => {
      if (resolvedTheme === 'dark') {
        root.classList.add('dark');
      } else {
        root.classList.remove('dark');
      }
    };

    if (theme === 'system') {
      // Use system preference
      const mediaQuery = window.matchMedia('(prefers-color-scheme: dark)');
      applyTheme(mediaQuery.matches ? 'dark' : 'light');

      // Listen for system preference changes
      const handleChange = (e: MediaQueryListEvent) => {
        applyTheme(e.matches ? 'dark' : 'light');
      };

      mediaQuery.addEventListener('change', handleChange);
      return () => mediaQuery.removeEventListener('change', handleChange);
    } else {
      // Use explicit theme setting
      applyTheme(theme);
    }
  }, [theme]);

  /**
   * Toggle between light and dark themes
   * If currently 'system', resolves to opposite of current system preference
   */
  const toggleTheme = () => {
    if (theme === 'light') {
      setTheme('dark');
    } else if (theme === 'dark') {
      setTheme('light');
    } else {
      // 'system' - toggle to opposite of current system preference
      const isDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
      setTheme(isDark ? 'light' : 'dark');
    }
  };

  /**
   * Get the resolved theme (never 'system')
   */
  const resolvedTheme: 'light' | 'dark' =
    theme === 'system'
      ? window.matchMedia('(prefers-color-scheme: dark)').matches
        ? 'dark'
        : 'light'
      : theme;

  return {
    theme,
    setTheme,
    toggleTheme,
    resolvedTheme,
    isDark: resolvedTheme === 'dark',
  };
}

export type { Theme };
