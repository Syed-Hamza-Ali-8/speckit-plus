import { useCallback } from 'react';
import { useLoginMutation, useRegisterMutation } from '@/services/authApi';
import type { LoginRequest, RegisterRequest, AuthResponse } from '@/types/auth';

/**
 * Auth state derived from localStorage and RTK Query
 */
export interface AuthState {
  isAuthenticated: boolean;
  token: string | null;
  isLoading: boolean;
  error: string | null;
}

/**
 * Auth hook return type
 */
export interface UseAuthReturn extends Omit<AuthState, 'error'> {
  login: (credentials: LoginRequest) => Promise<AuthResponse>;
  register: (data: RegisterRequest) => Promise<AuthResponse>;
  logout: () => void;
}

/**
 * useAuth hook
 * Provides authentication state and operations
 * Token is stored in localStorage for persistence
 */
export function useAuth(): UseAuthReturn {
  const [loginMutation, { isLoading: isLoginLoading }] = useLoginMutation();
  const [registerMutation, { isLoading: isRegisterLoading }] = useRegisterMutation();

  // Check if user is authenticated by checking localStorage
  const token = localStorage.getItem('token');
  const isAuthenticated = !!token;
  const isLoading = isLoginLoading || isRegisterLoading;

  /**
   * Login and store token
   */
  const login = useCallback(
    async (credentials: LoginRequest): Promise<AuthResponse> => {
      const result = await loginMutation(credentials).unwrap();
      localStorage.setItem('token', result.access_token);
      return result;
    },
    [loginMutation]
  );

  /**
   * Register and store token
   */
  const register = useCallback(
    async (data: RegisterRequest): Promise<AuthResponse> => {
      const result = await registerMutation(data).unwrap();
      localStorage.setItem('token', result.access_token);
      return result;
    },
    [registerMutation]
  );

  /**
   * Logout and clear token
   */
  const logout = useCallback(() => {
    localStorage.removeItem('token');
    window.location.href = '/login';
  }, []);

  return {
    isAuthenticated,
    token,
    isLoading,
    login,
    register,
    logout,
  };
}
