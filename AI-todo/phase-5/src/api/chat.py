"""Chat API endpoints for AI chatbot functionality with MCP tools."""
import time
import os
from typing import Optional
from uuid import UUID

from fastapi import APIRouter, HTTPException, Depends, Header
from sqlmodel import Session

from config.database import get_session
from core.security import decode_access_token
from agents.todo_agent import get_todo_agent
from chat.session import get_session_store, ChatMessage
from chat.schemas import ChatRequest, ChatResponse, ChatMetadata, ActionTakenResponse

router = APIRouter(prefix="/chat", tags=["chat"])

# Check if OpenAI is configured
AI_ENABLED = bool(os.getenv("OPENAI_API_KEY"))


@router.post("", response_model=ChatResponse)
async def chat_endpoint(
    body: ChatRequest,
    authorization: Optional[str] = Header(None),
    db: Session = Depends(get_session)
) -> ChatResponse:
    """Process a chat message using OpenAI with MCP tools.

    This endpoint uses OpenAI with MCP (Model Context Protocol) tools
    to process natural language requests for task management.

    Args:
        body: Chat request body with message and optional session_id.
        authorization: JWT token from Authorization header.
        db: Database session.

    Returns:
        ChatResponse with AI response and metadata.

    Raises:
        HTTPException: For various error conditions.
    """
    start_time = time.time()
    session_store = get_session_store()
    todo_agent = get_todo_agent()

    # Get authenticated user ID
    user_id = None
    if authorization and authorization.startswith("Bearer "):
        token = authorization.split(" ")[1]
        payload = decode_access_token(token)
        if payload:
            user_id_str = payload.get("sub")
            if user_id_str:
                try:
                    user_id = UUID(user_id_str)
                except ValueError:
                    pass

    # Check if OpenAI is enabled
    if not AI_ENABLED:
        raise HTTPException(
            status_code=503,
            detail="AI chatbot is not configured. Please set OPENAI_API_KEY environment variable."
        )

    if not user_id:
        raise HTTPException(
            status_code=401,
            detail="Please log in to use the chatbot."
        )

    # Get or create session
    session = session_store.get_or_create(
        user_id=user_id,
        session_id=body.session_id,
    )

    # Add user message to session
    user_message = ChatMessage(
        role="user",
        content=body.message,
    )
    session_store.add_message(session, user_message)

    # Get conversation history and clarification state from session
    # Reduced from 20 to 8 messages to stay within OpenRouter free tier token limits
    conversation_history = [
        {"role": msg.role, "content": msg.content}
        for msg in session.context.messages[-8:]  # Last 8 messages
    ]
    awaiting_clarification = session.context.awaiting_clarification

    # PRE-PROCESS: Check if user is asking to delete/update/complete a task
    # and if there are multiple matches, ask for clarification BEFORE calling OpenAI
    action_keywords = {
        "delete": ["delete", "remove"],
        "update": ["update", "change", "modify"],
        "complete": ["complete", "finish", "done", "mark as done"]
    }

    detected_action = None
    for action, keywords in action_keywords.items():
        if any(keyword in body.message.lower() for keyword in keywords):
            detected_action = action
            break

    # Check if we're awaiting clarification and the user is responding
    if awaiting_clarification and awaiting_clarification.get("matching_tasks"):
        # User is responding to clarification - match their response to a task
        matching_tasks = awaiting_clarification["matching_tasks"]
        user_response_lower = body.message.lower().strip()

        # Try to find exact or fuzzy match
        selected_task = None
        for task in matching_tasks:
            task_title_lower = task["title"].lower()
            # Check for exact match or if the user's response matches the task title
            if user_response_lower == task_title_lower:
                selected_task = task
                break
            # Check if user response is contained in task title or vice versa
            elif user_response_lower in task_title_lower or task_title_lower in user_response_lower:
                # Prefer exact case match if available
                if body.message.strip() == task["title"]:
                    selected_task = task
                    break
                elif not selected_task:  # Take first match if no exact case match
                    selected_task = task

        if selected_task:
            # Execute the pending action with the selected task
            pending_action = awaiting_clarification.get("action")

            if pending_action == "delete":
                # Import and execute delete_task directly
                from mcp_tools.server import delete_task, set_context

                set_context(db, user_id)
                result = await delete_task(task_id=selected_task["id"])

                response_text = f"I've deleted the task \"{selected_task['title']}\" for you."
                actions = [ActionTakenResponse(
                    tool="delete_task",
                    success=result.get("success", True),
                    summary=f"Deleted task: {selected_task['title']}"
                )]

                # Clear clarification state
                session.context.awaiting_clarification = None
                session_store.update(session)

                # Add assistant response to session
                assistant_message = ChatMessage(
                    role="assistant",
                    content=response_text,
                )
                session_store.add_message(session, assistant_message)

                # Calculate processing time
                processing_time_ms = int((time.time() - start_time) * 1000)

                return ChatResponse(
                    response=response_text,
                    session_id=session.session_id,
                    intent="delete",
                    actions=actions,
                    metadata=ChatMetadata(
                        processing_time_ms=processing_time_ms,
                        agent_chain=["Deterministic Clarification Handler"],
                        model=os.getenv("OPENAI_MODEL", "gpt-3.5-turbo")
                    )
                )

    if detected_action:
        print(f"[DEBUG] Detected action: {detected_action}")
        # Extract search term from message
        search_term = body.message.lower()
        for action_words in action_keywords.values():
            for word in action_words:
                search_term = search_term.replace(word, "")
        for word in ["the", "task", "a", "an", "my"]:
            search_term = search_term.replace(word, "")
        search_term = search_term.strip()
        print(f"[DEBUG] Extracted search term: '{search_term}'")

        if search_term:
            # Get all tasks and filter for matches
            from mcp_tools.server import list_tasks, set_context

            set_context(db, user_id)
            all_tasks = await list_tasks(status="pending")  # Get pending tasks only
            print(f"[DEBUG] Retrieved {len(all_tasks) if isinstance(all_tasks, list) else 0} tasks")
            print(f"[DEBUG] all_tasks type: {type(all_tasks)}")
            if isinstance(all_tasks, list) and len(all_tasks) > 0:
                print(f"[DEBUG] First task: {all_tasks[0]}")

            # list_tasks returns a list directly, not a dict
            if isinstance(all_tasks, list) and len(all_tasks) > 0:
                # Filter tasks that match the search term
                matching_tasks = [
                    task for task in all_tasks
                    if "error" not in task and (search_term in task["title"].lower() or task["title"].lower() in search_term)
                ]

                # If multiple matches, ask for clarification deterministically
                if len(matching_tasks) > 1:
                    # Build clarification message
                    task_list = []
                    for i, task in enumerate(matching_tasks, 1):
                        status_str = f"({task.get('status', 'unknown')})"
                        desc_str = f" - {task.get('description', '')[:50]}" if task.get('description') else ""
                        task_list.append(f"{i}. \"{task['title']}\" {status_str}{desc_str}")

                    response_text = f"I found {len(matching_tasks)} tasks matching '{search_term}'. Which one would you like to {detected_action}?\n\n" + "\n".join(task_list)

                    # Store clarification state in session
                    session.context.awaiting_clarification = {
                        "question": response_text,
                        "original_message": body.message,
                        "action": detected_action,
                        "matching_tasks": [
                            {
                                "id": str(task["id"]),
                                "title": task["title"],
                                "status": task.get("status", "unknown"),
                                "priority": task.get("priority", "unknown")
                            }
                            for task in matching_tasks
                        ]
                    }
                    session_store.update(session)

                    # Add assistant response to session
                    assistant_message = ChatMessage(
                        role="assistant",
                        content=response_text,
                    )
                    session_store.add_message(session, assistant_message)

                    # Calculate processing time
                    processing_time_ms = int((time.time() - start_time) * 1000)

                    return ChatResponse(
                        response=response_text,
                        session_id=session.session_id,
                        intent="chat",
                        actions=[],
                        metadata=ChatMetadata(
                            processing_time_ms=processing_time_ms,
                            agent_chain=["Deterministic Clarification Handler"],
                            model=os.getenv("OPENAI_MODEL", "gpt-3.5-turbo")
                        )
                    )

    # Process through Todo Agent (OpenAI with MCP tools)
    try:
        result = await todo_agent.process_message(
            message=body.message,
            user_id=user_id,
            db=db,
            conversation_history=conversation_history,
            awaiting_clarification=awaiting_clarification,
        )
    except Exception as e:
        # Log error for debugging
        print(f"Agent error: {str(e)}")
        import traceback
        print(traceback.format_exc())

        raise HTTPException(
            status_code=500,
            detail="An unexpected error occurred. Please try again."
        )

    # Extract response
    response_text = result.get("response", "I couldn't process that request.")

    # Add assistant message to session
    assistant_message = ChatMessage(
        role="assistant",
        content=response_text,
    )
    session_store.add_message(session, assistant_message)

    # Update clarification state if needed
    if result.get("awaiting_clarification"):
        session.context.awaiting_clarification = result["awaiting_clarification"]
    else:
        session.context.awaiting_clarification = None
    session_store.update(session)

    # Calculate processing time
    processing_time_ms = int((time.time() - start_time) * 1000)

    # Build actions from tool calls
    actions = []
    for tc in result.get("tool_calls", []):
        # Handle both list results (list_tasks) and dict results (other tools)
        tool_result = tc.get("result", {})
        if isinstance(tool_result, list):
            # list_tasks returns a list directly
            success = True
        elif isinstance(tool_result, dict):
            success = tool_result.get("success", True)
        else:
            success = True

        actions.append(
            ActionTakenResponse(
                tool=tc.get("type", "unknown"),
                success=success,
                summary=f"Executed {tc.get('type', 'tool')}"
            )
        )

    # Determine intent from tool calls or response content
    intent = "chat"
    if actions:
        tool_name = actions[0].tool.lower()
        if "list" in tool_name:
            intent = "read"
        elif "add" in tool_name or "create" in tool_name:
            intent = "create"
        elif "update" in tool_name:
            intent = "update"
        elif "delete" in tool_name:
            intent = "delete"
        elif "complete" in tool_name:
            intent = "complete"

    # Build response
    return ChatResponse(
        response=response_text,
        session_id=session.session_id,
        intent=intent,
        actions=actions,
        metadata=ChatMetadata(
            processing_time_ms=processing_time_ms,
            agent_chain=["TodoAgent (OpenAI)", "MCP Tools"],
            model=os.getenv("OPENAI_MODEL", "gpt-3.5-turbo")
        )
    )


@router.get("/info")
async def chat_info() -> dict:
    """Get information about the chat endpoint.

    Returns:
        dict with information about the chat system.
    """
    return {
        "version": "5.0",
        "description": "Chat endpoint using OpenAI with MCP tools",
        "model": os.getenv("OPENAI_MODEL", "gpt-3.5-turbo"),
        "features": [
            "Official MCP SDK integration",
            "OpenAI with function calling",
            "Session-based conversation history",
            "Deterministic clarification handling",
        ],
        "tools": [
            "add_task",
            "list_tasks",
            "complete_task",
            "delete_task",
            "update_task",
        ],
    }
