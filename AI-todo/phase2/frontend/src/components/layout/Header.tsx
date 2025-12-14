import { Link, useLocation } from 'react-router-dom';
import { CheckSquare } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { ThemeToggle } from './ThemeToggle';
import { useAuth } from '@/hooks/useAuth';
import { ROUTES } from '@/routes';

export interface HeaderProps {
  onMenuClick?: () => void;
}

/**
 * Header component
 * Navigation bar with logo, nav links, and theme toggle
 */
export function Header(_props: HeaderProps) {
  const { isAuthenticated, logout } = useAuth();
  const location = useLocation();

  const isAuthPage = location.pathname === ROUTES.LOGIN || location.pathname === ROUTES.REGISTER;

  return (
    <header className="sticky top-0 z-50 w-full border-b bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60">
      <div className="container mx-auto flex h-14 items-center px-4">
        {/* Logo */}
        <Link to={ROUTES.HOME} className="flex items-center gap-2 font-semibold">
          <CheckSquare className="h-5 w-5" />
          <span>Todo App</span>
        </Link>

        {/* Spacer */}
        <div className="flex-1" />

        {/* Navigation */}
        <nav className="flex items-center gap-2">
          {!isAuthenticated && !isAuthPage && (
            <>
              <Link to={ROUTES.LOGIN}>
                <Button variant="ghost" size="sm">
                  Login
                </Button>
              </Link>
              <Link to={ROUTES.REGISTER}>
                <Button size="sm">
                  Register
                </Button>
              </Link>
            </>
          )}

          {isAuthenticated && (
            <Button variant="ghost" size="sm" onClick={logout}>
              Logout
            </Button>
          )}

          <ThemeToggle />
        </nav>
      </div>
    </header>
  );
}
