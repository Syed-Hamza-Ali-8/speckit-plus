import { useState, useCallback, useEffect } from 'react';
import { toast } from 'sonner';
import { chatApiRaw } from '@/services/chatApi';
import type { ChatMessage, ActionTaken } from '@/types/chat';

const STORAGE_KEY_SESSION = 'chat_session_id';
const STORAGE_KEY_MESSAGES = 'chat_messages';

/**
 * Chat hook state
 */
export interface UseChatState {
  messages: ChatMessage[];
  sessionId: string | null;
  isLoading: boolean;
  error: string | null;
}

/**
 * Chat hook return type
 */
export interface UseChatReturn extends UseChatState {
  sendMessage: (content: string) => Promise<void>;
  clearMessages: () => void;
  onTasksChanged?: () => void;
}

/**
 * useChat hook - Manages chat state and API interactions
 *
 * Features:
 * - Maintains message history (persisted to localStorage)
 * - Tracks session ID for context (persisted to localStorage)
 * - Handles loading and error states
 * - Auto-scrolling via message updates
 * - Triggers task refresh on task operations
 */
export function useChat(onTasksChanged?: () => void): UseChatReturn {
  // Initialize from localStorage
  const [messages, setMessages] = useState<ChatMessage[]>(() => {
    try {
      const stored = localStorage.getItem(STORAGE_KEY_MESSAGES);
      return stored ? JSON.parse(stored) : [];
    } catch {
      return [];
    }
  });
  const [sessionId, setSessionId] = useState<string | null>(() => {
    return localStorage.getItem(STORAGE_KEY_SESSION);
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Persist messages to localStorage
  useEffect(() => {
    localStorage.setItem(STORAGE_KEY_MESSAGES, JSON.stringify(messages));
  }, [messages]);

  // Persist sessionId to localStorage
  useEffect(() => {
    if (sessionId) {
      localStorage.setItem(STORAGE_KEY_SESSION, sessionId);
    }
  }, [sessionId]);

  /**
   * Send a message to the chat API
   */
  const sendMessage = useCallback(
    async (content: string) => {
      if (!content.trim()) return;

      // Add user message immediately
      const userMessage: ChatMessage = {
        role: 'user',
        content: content.trim(),
        timestamp: new Date().toISOString(),
      };

      setMessages((prev) => [...prev, userMessage]);
      setIsLoading(true);
      setError(null);

      try {
        const response = await chatApiRaw.sendMessage(content, sessionId);

        // Update session ID
        setSessionId(response.session_id);

        // Add assistant message
        const assistantMessage: ChatMessage = {
          role: 'assistant',
          content: response.response,
          timestamp: new Date().toISOString(),
          actions: response.actions as ActionTaken[],
          intent: response.intent,
        };

        setMessages((prev) => [...prev, assistantMessage]);

        // Show toast for task operations and trigger refresh
        if (response.actions.length > 0) {
          const successActions = response.actions.filter((a) => a.success);
          if (successActions.length > 0) {
            // Check intent or action tool names for task operations
            const intent = response.intent;
            if (intent === 'create' || response.actions.some(a => a.tool.includes('add'))) {
              toast.success('Task created successfully');
              onTasksChanged?.();
            } else if (intent === 'complete' || response.actions.some(a => a.tool.includes('complete'))) {
              toast.success('Task marked as complete');
              onTasksChanged?.();
            } else if (intent === 'delete' || response.actions.some(a => a.tool.includes('delete'))) {
              toast.success('Task deleted');
              onTasksChanged?.();
            } else if (intent === 'update' || response.actions.some(a => a.tool.includes('update'))) {
              toast.success('Task updated');
              onTasksChanged?.();
            } else if (intent === 'read' || response.actions.some(a => a.tool.includes('list'))) {
              // List tasks - still trigger refresh to sync
              onTasksChanged?.();
            }
          }
        }
      } catch (err) {
        const errorMessage =
          err instanceof Error ? err.message : 'Failed to send message';
        setError(errorMessage);

        // Add error message to chat
        const errorChatMessage: ChatMessage = {
          role: 'assistant',
          content: `Sorry, something went wrong: ${errorMessage}. Please try again.`,
          timestamp: new Date().toISOString(),
        };

        setMessages((prev) => [...prev, errorChatMessage]);
        toast.error('Failed to send message');
      } finally {
        setIsLoading(false);
      }
    },
    [sessionId]
  );

  /**
   * Clear all messages and reset session
   */
  const clearMessages = useCallback(() => {
    setMessages([]);
    setSessionId(null);
    setError(null);
    localStorage.removeItem(STORAGE_KEY_MESSAGES);
    localStorage.removeItem(STORAGE_KEY_SESSION);
  }, []);

  return {
    messages,
    sessionId,
    isLoading,
    error,
    sendMessage,
    clearMessages,
  };
}
