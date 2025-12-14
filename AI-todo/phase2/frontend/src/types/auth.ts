/**
 * Authentication types for the frontend
 * Matches backend API contract from Phase 2
 */

/**
 * Login request payload
 */
export interface LoginRequest {
  email: string;
  password: string;
}

/**
 * Registration request payload
 */
export interface RegisterRequest {
  email: string;
  password: string;
}

/**
 * Authentication response from backend
 */
export interface AuthResponse {
  access_token: string;
  token_type: 'bearer';
}

/**
 * Decoded JWT payload (for client-side use)
 */
export interface JWTPayload {
  sub: string;      // User ID
  email: string;
  exp: number;      // Expiration timestamp
  iat: number;      // Issued at timestamp
}
