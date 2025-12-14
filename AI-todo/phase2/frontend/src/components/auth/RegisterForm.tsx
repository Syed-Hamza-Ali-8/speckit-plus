import { useState } from 'react';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { Eye, EyeOff } from 'lucide-react';
import { toast } from 'sonner';

import { registerSchema, type RegisterFormData } from '@/lib/validations/auth';
import { useRegisterMutation } from '@/services/authApi';
import { Input } from '@/components/ui/input';
import { Label } from '@/components/ui/label';
import { LoadingButton } from '@/components/ui/loading-button';
import { cn } from '@/lib/utils';

export interface RegisterFormProps {
  /** Callback fired after successful registration (auto-login) */
  onSuccess?: () => void;
}

/**
 * RegisterForm - React Hook Form + Zod validation + RTK Query
 * Features:
 * - Email/password/confirmPassword fields with inline validation
 * - Cross-field validation (password match)
 * - Password visibility toggles
 * - Loading state with spinner
 * - Toast notifications for success/error
 * - Auto-login after successful registration
 */
export function RegisterForm({ onSuccess }: RegisterFormProps) {
  const [showPassword, setShowPassword] = useState(false);
  const [showConfirmPassword, setShowConfirmPassword] = useState(false);
  const [register, { isLoading }] = useRegisterMutation();

  const {
    register: registerField,
    handleSubmit,
    formState: { errors },
  } = useForm<RegisterFormData>({
    resolver: zodResolver(registerSchema),
    mode: 'onBlur', // Validate on blur for real-time feedback
  });

  const onSubmit = async (data: RegisterFormData) => {
    try {
      // Only send email and password to API (not confirmPassword)
      const result = await register({
        email: data.email,
        password: data.password,
      }).unwrap();

      // Auto-login: Store token in localStorage
      localStorage.setItem('token', result.access_token);

      // Success toast (3 second duration - default)
      toast.success('Account created successfully!');

      // Call success callback for redirect
      onSuccess?.();
    } catch (error) {
      // Handle API errors with toast (5 second duration for errors)
      const apiError = error as { status?: number; data?: { detail?: string } };

      if (apiError.status === 409) {
        toast.error('Email already in use', { duration: 5000 });
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
          {...registerField('email')}
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
            autoComplete="new-password"
            disabled={isLoading}
            className={cn('pr-10', errors.password && 'border-destructive')}
            {...registerField('password')}
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

      {/* Confirm Password Field */}
      <div className="space-y-2">
        <Label htmlFor="confirmPassword">Confirm Password</Label>
        <div className="relative">
          <Input
            id="confirmPassword"
            type={showConfirmPassword ? 'text' : 'password'}
            placeholder="Confirm your password"
            autoComplete="new-password"
            disabled={isLoading}
            className={cn('pr-10', errors.confirmPassword && 'border-destructive')}
            {...registerField('confirmPassword')}
          />
          <button
            type="button"
            onClick={() => setShowConfirmPassword(!showConfirmPassword)}
            className="absolute right-3 top-1/2 -translate-y-1/2 text-muted-foreground hover:text-foreground transition-colors"
            tabIndex={-1}
            aria-label={showConfirmPassword ? 'Hide password' : 'Show password'}
          >
            {showConfirmPassword ? (
              <EyeOff className="h-4 w-4" />
            ) : (
              <Eye className="h-4 w-4" />
            )}
          </button>
        </div>
        {errors.confirmPassword && (
          <p className="text-sm text-destructive">{errors.confirmPassword.message}</p>
        )}
      </div>

      {/* Submit Button */}
      <LoadingButton
        type="submit"
        className="w-full"
        loading={isLoading}
        loadingText="Creating account..."
      >
        Create Account
      </LoadingButton>
    </form>
  );
}
