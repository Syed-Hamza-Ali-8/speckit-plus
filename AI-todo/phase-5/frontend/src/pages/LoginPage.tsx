import { Link, useNavigate } from 'react-router-dom';
import { Card, CardHeader, CardTitle, CardDescription, CardContent, CardFooter } from '@/components/ui/card';
import { LoginForm } from '@/components/auth/LoginForm';
import { useReturnUrl } from '@/hooks/useReturnUrl';
import { ROUTES } from '@/routes';

/**
 * LoginPage
 * Uses LoginForm component with React Hook Form + Zod validation
 * Supports return URL redirect after successful login
 */
export function LoginPage() {
  const navigate = useNavigate();
  const returnUrl = useReturnUrl();

  const handleSuccess = () => {
    // Immediate redirect to return URL (or /tasks)
    navigate(returnUrl, { replace: true });
  };

  return (
    <div className="min-h-[calc(100vh-3.5rem)] flex items-center justify-center p-4">
      <Card className="w-full max-w-md">
        <CardHeader className="space-y-1">
          <CardTitle className="text-2xl font-bold">Login</CardTitle>
          <CardDescription>
            Enter your credentials to access your tasks
          </CardDescription>
        </CardHeader>
        <CardContent>
          <LoginForm onSuccess={handleSuccess} />
        </CardContent>
        <CardFooter className="flex flex-col">
          <p className="text-sm text-muted-foreground text-center">
            Don't have an account?{' '}
            <Link to={ROUTES.REGISTER} className="text-primary hover:underline">
              Register
            </Link>
          </p>
        </CardFooter>
      </Card>
    </div>
  );
}
