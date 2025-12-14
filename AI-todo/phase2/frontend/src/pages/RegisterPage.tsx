import { Link, useNavigate } from 'react-router-dom';
import { Card, CardHeader, CardTitle, CardDescription, CardContent, CardFooter } from '@/components/ui/card';
import { RegisterForm } from '@/components/auth/RegisterForm';
import { ROUTES } from '@/routes';

/**
 * RegisterPage
 * Uses RegisterForm component with React Hook Form + Zod validation
 * Auto-login after successful registration, redirect to /tasks
 */
export function RegisterPage() {
  const navigate = useNavigate();

  const handleSuccess = () => {
    // Redirect to /tasks after successful registration (auto-login)
    navigate(ROUTES.TASKS, { replace: true });
  };

  return (
    <div className="min-h-[calc(100vh-3.5rem)] flex items-center justify-center p-4">
      <Card className="w-full max-w-md">
        <CardHeader className="space-y-1">
          <CardTitle className="text-2xl font-bold">Register</CardTitle>
          <CardDescription>
            Create an account to start managing your tasks
          </CardDescription>
        </CardHeader>
        <CardContent>
          <RegisterForm onSuccess={handleSuccess} />
        </CardContent>
        <CardFooter className="flex flex-col">
          <p className="text-sm text-muted-foreground text-center">
            Already have an account?{' '}
            <Link to={ROUTES.LOGIN} className="text-primary hover:underline">
              Login
            </Link>
          </p>
        </CardFooter>
      </Card>
    </div>
  );
}
