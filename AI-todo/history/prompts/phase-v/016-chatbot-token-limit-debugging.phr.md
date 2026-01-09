---
id: 016
title: Chatbot Token Limit Error Debugging
stage: misc
date: 2026-01-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: phase-v
branch: main
user: hamza
command: chatbot-debugging
labels: ["debugging", "chatbot", "openrouter", "token-limit", "api-error", "402-error"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - phase-5/src/api/chat.py (modified - reduced context from 20 to 8 messages)
  - phase-5/CHATBOT_ERROR_ANALYSIS.md (created)
  - phase-5/CHATBOT_FIX_INSTRUCTIONS.md (created)
tests:
  - Identified root cause through log analysis
  - Tested context window reduction (unsuccessful)
  - Verified baseline token usage exceeds free tier limit
---

## Prompt

I am trying to access chatbot and it is showing this: I encountered an error while processing your request. Please try again. while my input is: mark as complete the task API Test Recurring Task

## Response snapshot

Debugged chatbot error by analyzing backend logs and identified root cause: OpenRouter API free tier token limit exceeded (2347 tokens > 2248 limit).

**Investigation Process:**

1. **Initial Analysis:**
   - Checked backend logs: Found `openai.APIStatusError: Error code: 402`
   - Error message: "Prompt tokens limit exceeded: 2347 > 2248"
   - Provider: OpenRouter free tier
   - User command: "mark as complete the task API Test Recurring Task"

2. **First Fix Attempt:**
   - Hypothesis: Conversation history accumulation causing token overflow
   - Action: Reduced context window from 20 to 8 messages in chat.py:90
   - Rebuilt backend container
   - Result: ❌ FAILED - Still getting 402 error with same token count

3. **Root Cause Discovery:**
   - Tested with fresh session (only 1 message in history)
   - Token count still 2347 tokens
   - Conclusion: System prompt + MCP tool definitions consume >2000 tokens BEFORE any conversation
   - System prompt: 86 lines of detailed instructions
   - MCP tools: Multiple tools with full JSON schemas (add_task, list_tasks, update_task_by_name, delete_task_by_name, complete_task_by_name, create_recurring_task, etc.)

4. **Token Breakdown:**
   - System prompt: ~1500 tokens
   - MCP tool definitions: ~800 tokens
   - User message: ~50 tokens
   - Total baseline: ~2350 tokens
   - OpenRouter free tier limit: 2248 tokens
   - Overflow: 99+ tokens

**Solutions Provided:**

1. **Recommended: Switch to OpenAI API**
   - Token limit: 128,000 (57x more than OpenRouter)
   - Cost: ~$0.15 per 1M input tokens
   - Steps: Get API key, update .env, restart backend
   - Created detailed guide: CHATBOT_FIX_INSTRUCTIONS.md

2. **Alternative: Upgrade OpenRouter**
   - Add credits at https://openrouter.ai/settings/credits
   - No code changes needed
   - Similar pricing to OpenAI

3. **Not Recommended: Reduce System Prompt**
   - Would break chatbot functionality
   - Still might not provide enough headroom

**Documentation Created:**
- CHATBOT_ERROR_ANALYSIS.md: Comprehensive error analysis with all options
- CHATBOT_FIX_INSTRUCTIONS.md: Step-by-step guide to switch to OpenAI API

## Outcome

- ✅ Impact: Identified root cause of chatbot failure (OpenRouter free tier token limit). Provided clear solution path (switch to OpenAI API for 57x more token capacity). Created comprehensive documentation for user to implement fix.
- 🧪 Tests: Analyzed backend logs, tested context window reduction, verified baseline token usage with fresh session, confirmed system prompt + MCP tools exceed free tier limit
- 📁 Files: Modified chat.py (reduced context 20→8), created CHATBOT_ERROR_ANALYSIS.md and CHATBOT_FIX_INSTRUCTIONS.md with complete solution guides
- 🔁 Next prompts: User needs to get OpenAI API key and update .env file with new credentials, then restart backend to fix chatbot
- 🧠 Reflection: The context window reduction approach was logical but insufficient because the real bottleneck was the baseline token usage (system prompt + tools), not conversation history. This highlights the importance of testing with minimal context to isolate the true cause. OpenRouter's free tier (2248 tokens) is too restrictive for production chatbots with comprehensive tool definitions.

## Evaluation notes (flywheel)

- Failure modes observed: Initial hypothesis (conversation history) was incorrect, actual issue was baseline token usage from system prompt and MCP tool schemas exceeding free tier limit before any conversation
- Graders run and results (PASS/FAIL): Log analysis PASS (identified 402 error), Context reduction FAIL (didn't solve issue), Fresh session test PASS (isolated root cause), Token breakdown analysis PASS (confirmed baseline exceeds limit)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): User should switch to OpenAI API (128k token limit) rather than further reducing system prompt which would break functionality
