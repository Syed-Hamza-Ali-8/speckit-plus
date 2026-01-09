import type { ReactNode } from 'react';
import { Header } from './Header';
import { ChatWidget } from '@/components/chat/ChatWidget';
import { useAuth } from '@/hooks/useAuth';

export interface LayoutProps {
  children: ReactNode;
}

/**
 * Layout component
 * Application shell with Header and main content area
 * Includes ChatWidget for authenticated users
 */
export function Layout({ children }: LayoutProps) {
  const { isAuthenticated } = useAuth();

  return (
    <div className="min-h-screen bg-background">
      <Header />
      <main className="container mx-auto px-4 py-8">
        {children}
      </main>

      {/* Chat Widget - only shown for authenticated users */}
      {isAuthenticated && <ChatWidget />}
    </div>
  );
}
