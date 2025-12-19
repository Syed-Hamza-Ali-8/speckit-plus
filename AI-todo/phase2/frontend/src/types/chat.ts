/**
 * Chat-related type definitions for the AI chatbot interface
 */

/**
 * Intent types for user messages
 */
export type ChatIntent =
  | 'read'
  | 'create'
  | 'update'
  | 'delete'
  | 'complete'
  | 'plan'
  | 'chat';

/**
 * MCP tool names
 */
export type MCPTool =
  | 'list_tasks'
  | 'create_task'
  | 'update_task'
  | 'complete_task'
  | 'delete_task';

/**
 * Record of an MCP tool invocation
 */
export interface ActionTaken {
  tool: MCPTool;
  success: boolean;
  summary: string;
}

/**
 * Individual chat message
 */
export interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
  timestamp: string;
  actions?: ActionTaken[];
  intent?: ChatIntent;
}

/**
 * Chat request payload
 */
export interface ChatRequest {
  message: string;
  session_id?: string | null;
}

/**
 * Chat response metadata
 */
export interface ChatMetadata {
  processing_time_ms: number;
  agent_chain: string[];
  model: string;
}

/**
 * Chat response from API
 */
export interface ChatResponse {
  response: string;
  session_id: string;
  intent: ChatIntent;
  actions: ActionTaken[];
  metadata: ChatMetadata;
}

/**
 * Error response from chat API
 */
export interface ChatErrorResponse {
  error: true;
  code: string;
  message: string;
}

/**
 * WebSocket message types
 */
export type WSMessageType =
  | 'message'
  | 'ping'
  | 'token'
  | 'tool_start'
  | 'tool_end'
  | 'complete'
  | 'error'
  | 'pong';

/**
 * WebSocket incoming message (client to server)
 */
export interface WSIncomingMessage {
  type: 'message' | 'ping';
  content?: string;
  session_id?: string | null;
}

/**
 * WebSocket outgoing message (server to client)
 */
export interface WSOutgoingMessage {
  type: WSMessageType;
  content: string;
  metadata?: Record<string, unknown>;
}
