"""OpenAI function schemas for intent classification and entity extraction.

Defines the function calling schemas used by ConversationAgent
to classify user intents and extract task-related entities.
"""

from typing import Any

# Intent classification function schema
INTENT_CLASSIFICATION_FUNCTION: dict[str, Any] = {
    "name": "classify_intent",
    "description": "Classify the user's intent for a todo task management application",
    "parameters": {
        "type": "object",
        "properties": {
            "intent": {
                "type": "string",
                "enum": ["read", "create", "update", "delete", "complete", "plan", "chat"],
                "description": "The classified intent: read (view tasks), create (new task), update (modify task), delete (remove task), complete (mark done), plan (planning help), chat (general conversation)",
            },
            "confidence": {
                "type": "number",
                "description": "Confidence score from 0 to 1",
                "minimum": 0,
                "maximum": 1,
            },
            "task_reference": {
                "type": "string",
                "description": "Reference to a specific task if mentioned (e.g., 'the first one', 'buy groceries task')",
            },
        },
        "required": ["intent", "confidence"],
    },
}

# Task entity extraction function schema
TASK_EXTRACTION_FUNCTION: dict[str, Any] = {
    "name": "extract_task_data",
    "description": "Extract task-related entities from a user message",
    "parameters": {
        "type": "object",
        "properties": {
            "title": {
                "type": "string",
                "description": "The task title or action to be done",
                "maxLength": 200,
            },
            "description": {
                "type": "string",
                "description": "Additional details or context for the task",
                "maxLength": 1000,
            },
            "due_date": {
                "type": "string",
                "description": "Due date in YYYY-MM-DD format, or relative date like 'tomorrow', 'next week'",
            },
            "priority": {
                "type": "string",
                "enum": ["low", "medium", "high"],
                "description": "Task priority if mentioned",
            },
        },
        "required": [],
    },
}

# Combined intent + extraction function schema
INTENT_AND_EXTRACT_FUNCTION: dict[str, Any] = {
    "name": "classify_and_extract",
    "description": "Classify user intent AND extract any task-related entities in one call",
    "parameters": {
        "type": "object",
        "properties": {
            "intent": {
                "type": "string",
                "enum": ["read", "create", "update", "delete", "complete", "plan", "chat"],
                "description": "The classified intent",
            },
            "confidence": {
                "type": "number",
                "description": "Confidence score from 0 to 1",
                "minimum": 0,
                "maximum": 1,
            },
            "task_title": {
                "type": "string",
                "description": "Task title if creating or referencing a task",
            },
            "task_description": {
                "type": "string",
                "description": "Task description if provided",
            },
            "due_date": {
                "type": "string",
                "description": "Due date in YYYY-MM-DD format",
            },
            "task_reference": {
                "type": "string",
                "description": "Reference to existing task (e.g., 'that task', 'the first one')",
            },
        },
        "required": ["intent", "confidence"],
    },
}

# Confirmation detection function schema
CONFIRMATION_FUNCTION: dict[str, Any] = {
    "name": "detect_confirmation",
    "description": "Detect if the user is confirming or denying a pending action",
    "parameters": {
        "type": "object",
        "properties": {
            "is_confirmation": {
                "type": "boolean",
                "description": "True if user is confirming (yes, sure, do it, etc.)",
            },
            "is_denial": {
                "type": "boolean",
                "description": "True if user is denying (no, cancel, don't, etc.)",
            },
        },
        "required": ["is_confirmation", "is_denial"],
    },
}

# All available functions for OpenAI
AVAILABLE_FUNCTIONS = [
    INTENT_CLASSIFICATION_FUNCTION,
    TASK_EXTRACTION_FUNCTION,
    INTENT_AND_EXTRACT_FUNCTION,
    CONFIRMATION_FUNCTION,
]

# System prompts for different agent roles
SYSTEM_PROMPTS = {
    "intent_classifier": """You are an intent classifier for a todo task management application.
Analyze the user's message and classify their intent into one of these categories:

- read: User wants to view, list, or see their tasks (e.g., "show my tasks", "what do I have to do")
- create: User wants to create a new task (e.g., "add a task", "remind me to", "I need to")
- update: User wants to modify an existing task (e.g., "change", "edit", "rename")
- delete: User wants to remove a task (e.g., "delete", "remove", "get rid of")
- complete: User wants to mark a task as done (e.g., "done", "finished", "complete")
- plan: User wants planning help (e.g., "help me plan", "prioritize", "what should I do first")
- chat: General conversation not about tasks (e.g., "hello", "how are you", "thanks")

Be concise and accurate. When in doubt, default to 'chat' with lower confidence.""",
    "task_extractor": """You are a task data extractor for a todo application.
From the user's message, extract:
- title: The main task action (required for create)
- description: Any additional details (optional)
- due_date: When it's due in YYYY-MM-DD format (optional)

For relative dates like "tomorrow" or "next Monday", convert to actual dates.
Be concise and only extract what's explicitly mentioned.""",
    "confirmation_detector": """You detect if a user is confirming or denying a pending action.
Confirmation phrases: yes, sure, do it, confirm, okay, go ahead, please, definitely
Denial phrases: no, cancel, don't, stop, nevermind, wait, hold on

Return true for is_confirmation if they're agreeing.
Return true for is_denial if they're refusing.""",
}
