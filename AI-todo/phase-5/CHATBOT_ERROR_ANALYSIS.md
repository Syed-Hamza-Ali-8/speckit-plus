================================================================================
CHATBOT ERROR ANALYSIS AND SOLUTIONS
Phase-5 Todo Application - OpenRouter API Token Limit Issue
================================================================================

Date: 2026-01-09
Issue: Chatbot failing with "I encountered an error while processing your request"
User Input: "mark as complete the task API Test Recurring Task"

================================================================================
ROOT CAUSE IDENTIFIED
================================================================================

Error Type: OpenRouter API Token Limit Exceeded
Error Code: 402 (Payment Required)

Full Error Message:
```
openai.APIStatusError: Error code: 402 - {
  'error': {
    'message': 'Prompt tokens limit exceeded: 2347 > 2248.
                To increase, visit https://openrouter.ai/settings/credits
                and upgrade to a paid account',
    'code': 402,
    'metadata': {'provider_name': None}
  },
  'user_id': 'user_2xmt0j6xFq3KINwGjLQ8oyrsSXt'
}
```

Current Configuration:
- API Provider: OpenRouter (https://openrouter.ai/api/v1)
- Model: openai/gpt-4o-mini
- Token Limit: 2248 tokens (free tier)
- Current Request: 2347 tokens (99 tokens over limit)

================================================================================
WHY THIS IS HAPPENING
================================================================================

1. **Conversation History Accumulation**
   - The chatbot maintains last 20 messages in context
   - Each message adds tokens to the request
   - Your conversation history has grown beyond the free tier limit

2. **MCP Tools Context**
   - The chatbot includes MCP tool definitions in each request
   - Tool schemas add significant token overhead
   - Multiple tools (ReadUserTasks, ModifyUserTasks, etc.) increase token count

3. **Free Tier Limitation**
   - OpenRouter free tier: 2248 tokens max
   - Your request: 2347 tokens (104% of limit)
   - Need 99 more tokens to process this request

================================================================================
SOLUTION OPTIONS
================================================================================

OPTION 1: Upgrade OpenRouter Account (Recommended for Production)
────────────────────────────────────────────────────────────────────────────
✅ Pros:
   - Immediate fix
   - Higher token limits
   - Better performance
   - Production-ready

❌ Cons:
   - Requires payment
   - Ongoing cost

Steps:
1. Visit: https://openrouter.ai/settings/credits
2. Add credits or upgrade to paid plan
3. No code changes needed

Cost: ~$0.10-$1.00 per 1M tokens (varies by model)


OPTION 2: Switch to Direct OpenAI API (Best for Development)
────────────────────────────────────────────────────────────────────────────
✅ Pros:
   - Higher token limits (128k for gpt-4o-mini)
   - More reliable
   - Better documentation
   - No intermediary service

❌ Cons:
   - Requires OpenAI API key
   - Slightly different pricing

Steps:
1. Get OpenAI API key from: https://platform.openai.com/api-keys
2. Update environment variables:
   ```bash
   # In phase-5/.env file
   OPENAI_API_KEY=sk-proj-your-openai-key-here
   OPENAI_BASE_URL=https://api.openai.com/v1
   OPENAI_MODEL=gpt-4o-mini
   ```
3. Restart backend container:
   ```bash
   docker restart todo-backend
   ```

Cost: ~$0.15 per 1M input tokens, ~$0.60 per 1M output tokens


OPTION 3: Use Google Gemini (Free Alternative)
────────────────────────────────────────────────────────────────────────────
✅ Pros:
   - Free tier available
   - High token limits (1M tokens for Gemini 1.5 Flash)
   - Good performance

❌ Cons:
   - Requires code changes to support Gemini API
   - Different API format

Steps:
1. Get Gemini API key from: https://makersuite.google.com/app/apikey
2. Requires code modifications to support Gemini API format
3. Not immediately compatible with current OpenAI SDK usage


OPTION 4: Reduce Context Window (Quick Workaround)
────────────────────────────────────────────────────────────────────────────
✅ Pros:
   - No cost
   - Immediate fix
   - No API key changes needed

❌ Cons:
   - Reduces chatbot memory
   - May affect conversation quality
   - Temporary solution

Steps:
1. Modify chat.py to reduce conversation history:
   ```python
   # Change line 90 from:
   for msg in session.context.messages[-20:]  # Last 20 messages

   # To:
   for msg in session.context.messages[-10:]  # Last 10 messages
   ```

2. Restart backend:
   ```bash
   docker restart todo-backend
   ```


OPTION 5: Clear Chat Session (Immediate Workaround)
────────────────────────────────────────────────────────────────────────────
✅ Pros:
   - Instant fix
   - No code changes
   - No cost

❌ Cons:
   - Loses conversation history
   - Temporary solution

Steps:
1. Refresh the page or start a new chat session
2. The chatbot will start with empty context
3. Try your command again: "mark as complete the task API Test Recurring Task"


================================================================================
RECOMMENDED IMMEDIATE ACTION
================================================================================

For Development/Testing:
→ Use OPTION 5 (Clear Session) + OPTION 2 (Switch to OpenAI)

For Production:
→ Use OPTION 1 (Upgrade OpenRouter) or OPTION 2 (Switch to OpenAI)


================================================================================
STEP-BY-STEP: SWITCH TO OPENAI API (RECOMMENDED)
================================================================================

1. Get OpenAI API Key:
   - Visit: https://platform.openai.com/api-keys
   - Click "Create new secret key"
   - Copy the key (starts with sk-proj-...)

2. Update Environment File:
   ```bash
   cd /mnt/d/hamza/speckit-plus/AI-todo/phase-5

   # Edit .env file
   nano .env

   # Update these lines:
   OPENAI_API_KEY=sk-proj-YOUR-KEY-HERE
   OPENAI_BASE_URL=https://api.openai.com/v1
   OPENAI_MODEL=gpt-4o-mini
   ```

3. Restart Backend:
   ```bash
   docker restart todo-backend
   ```

4. Test Chatbot:
   - Refresh the frontend page
   - Try: "mark as complete the task API Test Recurring Task"
   - Should work with much higher token limit (128k tokens)


================================================================================
QUICK FIX: CLEAR SESSION NOW
================================================================================

To immediately fix the issue without any changes:

1. In the frontend, refresh the page (F5 or Ctrl+R)
2. This will start a new chat session with empty context
3. Try your command again: "mark as complete the task API Test Recurring Task"

This should work because the new session will have fewer tokens in context.


================================================================================
MONITORING AND PREVENTION
================================================================================

To prevent this issue in the future:

1. **Add Token Monitoring**
   - Log token usage in chat endpoint
   - Alert when approaching limits

2. **Implement Session Cleanup**
   - Auto-clear old sessions
   - Limit conversation history length

3. **Add Error Handling**
   - Catch 402 errors
   - Show user-friendly message: "Chat history too long, starting new session"

4. **Use Appropriate Model**
   - gpt-4o-mini: 128k context (recommended)
   - gpt-3.5-turbo: 16k context
   - gpt-4: 8k context


================================================================================
SUMMARY
================================================================================

Issue: OpenRouter free tier token limit exceeded (2347 > 2248 tokens)

Immediate Fix: Refresh page to clear chat session

Best Long-term Solution: Switch to direct OpenAI API
- Higher limits (128k tokens)
- More reliable
- Better for production

Alternative: Upgrade OpenRouter account at https://openrouter.ai/settings/credits

================================================================================
