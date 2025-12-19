"""In-memory session store for chat conversations.

Provides session management with TTL-based expiration
and context preservation across messages.
"""

from datetime import datetime, timedelta
from functools import lru_cache
from uuid import UUID, uuid4

from pydantic import BaseModel, Field

from app.agents.base import ChatMessage, SessionContext
from app.core.config import get_settings


class ChatSession(BaseModel):
    """Session state for a chat conversation.

    Attributes:
        session_id: Unique session identifier.
        user_id: Owner's user ID.
        created_at: Session creation timestamp.
        last_activity: Last message timestamp.
        context: Conversation context.
    """

    session_id: UUID
    user_id: UUID
    created_at: datetime
    last_activity: datetime
    context: SessionContext = Field(default_factory=SessionContext)


class InMemorySessionStore:
    """In-memory session storage with TTL expiration.

    Sessions are stored in a dictionary and automatically
    expire after the configured TTL (default 30 minutes).
    """

    def __init__(self) -> None:
        """Initialize the session store."""
        self._sessions: dict[UUID, ChatSession] = {}
        self._settings = get_settings()

    @property
    def ttl_seconds(self) -> int:
        """Get TTL in seconds."""
        return self._settings.session_ttl_minutes * 60

    def get(self, session_id: UUID) -> ChatSession | None:
        """Get a session by ID if it exists and hasn't expired.

        Args:
            session_id: Session identifier.

        Returns:
            ChatSession if found and valid, None otherwise.
        """
        session = self._sessions.get(session_id)
        if session is None:
            return None

        # Check expiration
        if self._is_expired(session):
            del self._sessions[session_id]
            return None

        return session

    def get_or_create(self, user_id: UUID, session_id: UUID | None = None) -> ChatSession:
        """Get existing session or create a new one.

        Args:
            user_id: Owner's user ID.
            session_id: Optional existing session ID.

        Returns:
            Existing or newly created ChatSession.
        """
        if session_id:
            session = self.get(session_id)
            if session and session.user_id == user_id:
                return session

        return self.create(user_id)

    def create(self, user_id: UUID) -> ChatSession:
        """Create a new session for a user.

        Args:
            user_id: Owner's user ID.

        Returns:
            Newly created ChatSession.
        """
        now = datetime.utcnow()
        session = ChatSession(
            session_id=uuid4(),
            user_id=user_id,
            created_at=now,
            last_activity=now,
            context=SessionContext(),
        )
        self._sessions[session.session_id] = session
        return session

    def update(self, session: ChatSession) -> None:
        """Update session with new activity timestamp.

        Args:
            session: Session to update.
        """
        session.last_activity = datetime.utcnow()
        self._sessions[session.session_id] = session

    def delete(self, session_id: UUID) -> bool:
        """Delete a session.

        Args:
            session_id: Session to delete.

        Returns:
            True if deleted, False if not found.
        """
        if session_id in self._sessions:
            del self._sessions[session_id]
            return True
        return False

    def add_message(self, session: ChatSession, message: ChatMessage) -> None:
        """Add a message to session context with rolling window.

        Args:
            session: Session to add message to.
            message: Message to add.
        """
        max_messages = self._settings.session_max_messages
        session.context.messages.append(message)

        # Maintain rolling window
        if len(session.context.messages) > max_messages:
            session.context.messages = session.context.messages[-max_messages:]

        self.update(session)

    def cleanup_expired(self) -> int:
        """Remove all expired sessions.

        Returns:
            Number of sessions removed.
        """
        expired_ids = [
            sid
            for sid, session in self._sessions.items()
            if self._is_expired(session)
        ]
        for sid in expired_ids:
            del self._sessions[sid]
        return len(expired_ids)

    def _is_expired(self, session: ChatSession) -> bool:
        """Check if a session has expired.

        Args:
            session: Session to check.

        Returns:
            True if expired, False otherwise.
        """
        expiry_time = session.last_activity + timedelta(seconds=self.ttl_seconds)
        return datetime.utcnow() > expiry_time

    @property
    def active_session_count(self) -> int:
        """Get count of non-expired sessions."""
        self.cleanup_expired()
        return len(self._sessions)


# Singleton instance
_session_store: InMemorySessionStore | None = None


@lru_cache
def get_session_store() -> InMemorySessionStore:
    """Get singleton session store instance.

    Returns:
        InMemorySessionStore instance.
    """
    global _session_store
    if _session_store is None:
        _session_store = InMemorySessionStore()
    return _session_store
