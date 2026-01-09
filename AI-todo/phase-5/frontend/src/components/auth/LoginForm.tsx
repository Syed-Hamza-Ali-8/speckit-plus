import { useState } from 'react';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { Eye, EyeOff } from 'lucide-react';
import { toast } from 'sonner';

import { loginSchema, type LoginFormData } from '@/lib/validations/auth';
import { useLoginMutation } from '@/services/authApi';
import { Input } from '@/components/ui/input';
import { Label } from '@/components/ui/label';
import { LoadingButton } from '@/components/ui/loading-button';
import { cn } from '@/lib/utils';

export interface LoginFormProps {
  /** Callback fired after successful login */
  onSuccess?: () => void;
}

/**
 * LoginForm - React Hook Form + Zod validation + RTK Query
 * Features:
 * - Email/password fields with inline validation
 * - Password visibility toggle
 * - Loading state with spinner
 * - Toast notifications for success/error
 */
export function LoginForm({ onSuccess }: LoginFormProps) {
  const [showPassword, setShowPassword] = useState(false);
  const [login, { isLoading }] = useLoginMutation();

  const {
    register,
    handleSubmit,
    formState: { errors },
  } = useForm<LoginFormData>({
    resolver: zodResolver(loginSchema),
    mode: 'onBlur', // Validate on blur for real-time feedback
  });

  const onSubmit = async (data: LoginFormData) => {
    try {
      const result = await login(data).unwrap();

      // Store token in localStorage
      localStorage.setItem('token', result.access_token);

      // Success toast (3 second duration - default)
      toast.success('Welcome back!');

      // Call success callback for redirect
      onSuccess?.();
    } catch (error) {
      // Handle API errors with toast (5 second duration for errors)
      const apiError = error as { status?: number; data?: { detail?: string } };

      if (apiError.status === 401) {
        toast.error('Invalid email or password', { duration: 5000 });
      } else if (apiError.data?.detail) {
        toast.error(apiError.data.detail, { duration: 5000 });
      } else {
        toast.error('Connection error. Please try again.', { duration: 5000 });
      }
    }
  };

  return (
    <form onSubmit={handleSubmit(onSubmit)} className="space-y-4">
      {/* Email Field */}
      <div className="space-y-2">
        <Label htmlFor="email">Email</Label>
        <Input
          id="email"
          type="email"
          placeholder="Enter your email"
          autoComplete="email"
          disabled={isLoading}
          className={cn(errors.email && 'border-destructive')}
          {...register('email')}
        />
        {errors.email && (
          <p className="text-sm text-destructive">{errors.email.message}</p>
        )}
      </div>

      {/* Password Field */}
      <div className="space-y-2">
        <Label htmlFor="password">Password</Label>
        <div className="relative">
          <Input
            id="password"
            type={showPassword ? 'text' : 'password'}
            placeholder="Enter your password"
            autoComplete="current-password"
            disabled={isLoading}
            className={cn('pr-10', errors.password && 'border-destructive')}
            {...register('password')}
          />
          <button
            type="button"
            onClick={() => setShowPassword(!showPassword)}
            className="absolute right-3 top-1/2 -translate-y-1/2 text-muted-foreground hover:text-foreground transition-colors"
            tabIndex={-1}
            aria-label={showPassword ? 'Hide password' : 'Show password'}
          >
            {showPassword ? (
              <EyeOff className="h-4 w-4" />
            ) : (
              <Eye className="h-4 w-4" />
            )}
          </button>
        </div>
        {errors.password && (
          <p className="text-sm text-destructive">{errors.password.message}</p>
        )}
      </div>

      {/* Submit Button */}
      <LoadingButton
        type="submit"
        className="w-full"
        loading={isLoading}
        loadingText="Logging in..."
      >
        Login
      </LoadingButton>
    </form>
  );
}
