"""Chat module for AI-powered task management.

Provides the chat API endpoints, session management,
and WebSocket streaming support.
"""

from app.chat.router import router as chat_router
from app.chat.websocket import router as websocket_router
from app.chat.schemas import ChatMetadata, ChatRequest, ChatResponse
from app.chat.session import ChatSession, InMemorySessionStore, get_session_store

__all__ = [
    "chat_router",
    "websocket_router",
    "ChatRequest",
    "ChatResponse",
    "ChatMetadata",
    "ChatSession",
    "InMemorySessionStore",
    "get_session_store",
]
