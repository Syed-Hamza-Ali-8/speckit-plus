# Example: Chatkit Integration for Todo AI Chatbot

## Goal

Provide a chat interface for the Todo AI Chatbot using Chatkit.

---

## Scenario

User opens the `/chat` page and types:

"What tasks do I have today?"

---

## What Happens

1. Chatkit captures the user message
2. Chatkit sends the message to `/api/chat`
3. Backend guardrails validate the request
4. OpenAI Agent (via SDK) runs with tools
5. Agent returns a structured Result
6. Backend sends the response back
7. Chatkit renders the assistant reply

---

## Outcome

The user interacts with an AI-powered chatbot
without any custom chat UI implementation.
