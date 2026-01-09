# FastAPI Error Patterns

Use HTTPException for known errors.
Use global handlers for unexpected errors.
Return JSON objects with shape:
{
  "error": {
      "type": "...",
      "message": "...",
      "detail": {...}
  }
}
