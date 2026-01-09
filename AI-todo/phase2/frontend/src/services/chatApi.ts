import { api, TAG_TYPES } from './api';
import type { ChatRequest, ChatResponse } from '@/types/chat';

/**
 * Chat API endpoints using RTK Query
 */
export const chatApi = api.injectEndpoints({
  endpoints: (builder) => ({
    /**
     * Send a chat message
     * POST /chat
     */
    sendMessage: builder.mutation<ChatResponse, ChatRequest>({
      query: (body) => ({
        url: '/chat',
        method: 'POST',
        body,
      }),
      // Invalidate tasks cache when chat operations modify tasks
      invalidatesTags: (result) => {
        if (!result) return [];
        // Only invalidate if a task-modifying action was taken
        const modifyingActions = ['create_task', 'update_task', 'complete_task', 'delete_task'];
        const shouldInvalidate = result.actions.some((a) =>
          modifyingActions.includes(a.tool) && a.success
        );
        return shouldInvalidate ? [{ type: TAG_TYPES.Task, id: 'LIST' }] : [];
      },
    }),
  }),
});

export const { useSendMessageMutation } = chatApi;

/**
 * Raw fetch-based chat API for non-RTK usage
 * Useful for streaming or standalone components
 */
export const chatApiRaw = {
  /**
   * Send a chat message using fetch
   */
  async sendMessage(message: string, sessionId?: string | null): Promise<ChatResponse> {
    const token = localStorage.getItem('token');
    const baseUrl = import.meta.env.VITE_API_URL || '/api';

    const response = await fetch(`${baseUrl}/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        ...(token ? { Authorization: `Bearer ${token}` } : {}),
      },
      body: JSON.stringify({
        message,
        session_id: sessionId,
      }),
    });

    if (!response.ok) {
      const error = await response.json().catch(() => ({ message: 'Unknown error' }));
      throw new Error(error.message || `HTTP ${response.status}`);
    }

    return response.json();
  },

  /**
   * Get WebSocket URL for streaming chat
   */
  getWebSocketUrl(): string {
    const token = localStorage.getItem('token');
    const baseUrl = import.meta.env.VITE_API_URL || '';

    // Convert HTTP URL to WS URL
    const wsBase = baseUrl
      .replace(/^http:/, 'ws:')
      .replace(/^https:/, 'wss:')
      .replace(/\/api$/, '');

    return `${wsBase}/chat/stream?token=${token}`;
  },
};
