import type { ReactNode } from 'react';
import { Header } from './Header';

export interface LayoutProps {
  children: ReactNode;
}

/**
 * Layout component
 * Application shell with Header and main content area
 */
export function Layout({ children }: LayoutProps) {
  return (
    <div className="min-h-screen bg-background">
      <Header />
      <main className="container mx-auto px-4 py-8">
        {children}
      </main>
    </div>
  );
}
