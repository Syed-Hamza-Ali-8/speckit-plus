"""WebSocket handler for real-time chat streaming.

Provides WebSocket endpoint for streaming chat responses
with token-by-token delivery and tool event notifications.
"""

import json
from uuid import UUID

from fastapi import APIRouter, Depends, Query, WebSocket, WebSocketDisconnect
from jose import JWTError, jwt
from sqlmodel.ext.asyncio.session import AsyncSession

from app.agents.base import AgentRequest, ChatMessage
from app.agents.router import get_agent_router
from app.core.database import get_async_session
from app.chat.schemas import WSIncomingMessage, WSOutgoingMessage
from app.chat.session import get_session_store
from app.core.config import get_settings
from app.models.user import User

router = APIRouter()


async def get_user_from_token(token: str) -> User | None:
    """Validate JWT token and return user.

    Args:
        token: JWT token string.

    Returns:
        User if valid, None otherwise.
    """
    settings = get_settings()
    try:
        payload = jwt.decode(
            token,
            settings.jwt_secret_key,
            algorithms=[settings.jwt_algorithm],
        )
        user_id = payload.get("sub")
        if user_id is None:
            return None

        # Create a minimal user object for WebSocket
        # In production, you'd fetch from DB
        return User(
            id=UUID(user_id),
            email="",
            password_hash="",
        )
    except JWTError:
        return None


@router.websocket("/chat/stream")
async def websocket_chat(
    websocket: WebSocket,
    token: str = Query(...),
    db: AsyncSession = Depends(get_async_session),
) -> None:
    """WebSocket endpoint for streaming chat.

    Accepts messages from client and streams responses with:
    - token: Individual response tokens
    - tool_start: When an MCP tool begins execution
    - tool_end: When an MCP tool completes
    - complete: Final response with metadata
    - error: Error messages
    - pong: Response to ping

    Args:
        websocket: WebSocket connection.
        token: JWT token for authentication (query param).
        db: Database session.
    """
    # Authenticate
    user = await get_user_from_token(token)
    if not user:
        await websocket.close(code=4001, reason="Unauthorized")
        return

    await websocket.accept()

    session_store = get_session_store()
    agent_router = get_agent_router()
    current_session_id: UUID | None = None

    try:
        while True:
            # Receive message
            data = await websocket.receive_text()

            try:
                incoming = WSIncomingMessage.model_validate_json(data)
            except Exception:
                await _send_error(websocket, "Invalid message format", "INVALID_FORMAT")
                continue

            # Handle ping
            if incoming.type == "ping":
                await websocket.send_json(
                    WSOutgoingMessage(
                        type="pong",
                        content="",
                    ).model_dump()
                )
                continue

            # Handle chat message
            if incoming.type == "message":
                if not incoming.content:
                    await _send_error(websocket, "Message content required", "EMPTY_MESSAGE")
                    continue

                # Get or create session
                session = session_store.get_or_create(
                    user_id=user.id,
                    session_id=incoming.session_id or current_session_id,
                )
                current_session_id = session.session_id

                # Add user message
                user_message = ChatMessage(
                    role="user",
                    content=incoming.content,
                )
                session_store.add_message(session, user_message)

                # Build agent request
                agent_request = AgentRequest(
                    intent="",
                    user_id=user.id,
                    session_id=session.session_id,
                    message=incoming.content,
                    context=session.context,
                )

                try:
                    # Process and stream response
                    response, agent_chain = await agent_router.route_with_context(
                        agent_request, db
                    )

                    # Send tool events
                    for action in response.actions_taken:
                        await websocket.send_json(
                            WSOutgoingMessage(
                                type="tool_end",
                                content="",
                                metadata={
                                    "tool": action.tool,
                                    "success": action.success,
                                    "summary": action.summary,
                                },
                            ).model_dump()
                        )

                    # Send complete response
                    await websocket.send_json(
                        WSOutgoingMessage(
                            type="complete",
                            content=response.message,
                            metadata={
                                "session_id": str(session.session_id),
                                "intent": agent_request.intent or "chat",
                                "agent_chain": agent_chain,
                            },
                        ).model_dump()
                    )

                    # Add assistant message to session
                    assistant_message = ChatMessage(
                        role="assistant",
                        content=response.message,
                    )
                    session_store.add_message(session, assistant_message)

                except Exception as e:
                    await _send_error(
                        websocket,
                        f"Processing error: {str(e)}",
                        "PROCESSING_ERROR",
                    )

    except WebSocketDisconnect:
        # Client disconnected - cleanup if needed
        pass
    except Exception as e:
        # Unexpected error
        try:
            await _send_error(websocket, str(e), "INTERNAL_ERROR")
        except Exception:
            pass


async def _send_error(
    websocket: WebSocket,
    message: str,
    code: str,
) -> None:
    """Send an error message through WebSocket.

    Args:
        websocket: WebSocket connection.
        message: Error message.
        code: Error code.
    """
    await websocket.send_json(
        WSOutgoingMessage(
            type="error",
            content=message,
            metadata={"code": code},
        ).model_dump()
    )
