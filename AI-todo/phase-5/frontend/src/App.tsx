import { useEffect } from 'react';
import { BrowserRouter, Routes, Route, Navigate } from 'react-router-dom';
import { Toaster } from 'sonner';
import { useTheme } from '@/hooks/useTheme';
import { ROUTES } from '@/routes';
import { ProtectedRoute } from '@/routes/ProtectedRoute';
import { Layout } from '@/components/layout/Layout';
import { LoginPage } from '@/pages/LoginPage';
import { RegisterPage } from '@/pages/RegisterPage';
import { TasksPage } from '@/pages/TasksPage';
import { LandingPage } from '@/pages/LandingPage';
import { ProfilePage } from '@/pages/ProfilePage';
import { SettingsPage } from '@/pages/SettingsPage';
import { ChatPage } from '@/pages/ChatPage';
import { RecurringTaskPatternsPage } from '@/pages/RecurringTaskPatternsPage';
import { useAuth } from '@/hooks/useAuth';

function App() {
  const { theme, isDark } = useTheme();
  const { isAuthenticated } = useAuth();

  // Log theme value for verification
  useEffect(() => {
    console.log('[Zustand] Current theme:', theme, '| isDark:', isDark);
  }, [theme, isDark]);

  return (
    <BrowserRouter>
      {/* Sonner Toaster - positioned top-right, respects dark mode */}
      <Toaster
        position="top-right"
        theme={isDark ? 'dark' : 'light'}
        richColors
        closeButton
        toastOptions={{
          duration: 3000,
        }}
      />
      <Layout>
        <Routes>
          {/* Landing page for non-authenticated users */}
          <Route
            path={ROUTES.HOME}
            element={
              isAuthenticated ? (
                <Navigate to={ROUTES.TASKS} replace />
              ) : (
                <LandingPage />
              )
            }
          />

          {/* Public routes */}
          <Route path={ROUTES.LOGIN} element={<LoginPage />} />
          <Route path={ROUTES.REGISTER} element={<RegisterPage />} />

          {/* Protected routes */}
          <Route
            path={ROUTES.TASKS}
            element={
              <ProtectedRoute>
                <TasksPage />
              </ProtectedRoute>
            }
          />
          <Route
            path={ROUTES.PROFILE}
            element={
              <ProtectedRoute>
                <ProfilePage />
              </ProtectedRoute>
            }
          />
          <Route
            path={ROUTES.SETTINGS}
            element={
              <ProtectedRoute>
                <SettingsPage />
              </ProtectedRoute>
            }
          />
          <Route
            path={ROUTES.CHAT}
            element={
              <ProtectedRoute>
                <ChatPage />
              </ProtectedRoute>
            }
          />
          <Route
            path={ROUTES.RECURRING_PATTERNS}
            element={
              <ProtectedRoute>
                <RecurringTaskPatternsPage />
              </ProtectedRoute>
            }
          />

          {/* Catch-all redirect */}
          <Route path="*" element={<Navigate to={ROUTES.HOME} replace />} />
        </Routes>
      </Layout>
    </BrowserRouter>
  );
}

export default App;
