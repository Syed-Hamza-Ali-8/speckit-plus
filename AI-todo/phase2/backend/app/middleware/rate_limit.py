"""Rate limiting configuration using slowapi.

Supports per-user rate limiting (by JWT user ID) with IP fallback
for unauthenticated requests.
"""

from slowapi import Limiter
from slowapi.util import get_remote_address
from starlette.requests import Request


def get_user_rate_limit_key(request: Request) -> str:
    """Extract rate limit key: user ID from JWT if authenticated, else IP.

    This prevents bypass via multiple accounts while still protecting
    unauthenticated endpoints from abuse.

    Args:
        request: The incoming HTTP request.

    Returns:
        Rate limit key string (either "user:{uuid}" or IP address).
    """
    # Check if user is authenticated (set by auth dependency)
    if hasattr(request.state, "user") and request.state.user:
        return f"user:{request.state.user.id}"
    return get_remote_address(request)


# Create limiter instance using user-based key function
limiter = Limiter(key_func=get_user_rate_limit_key)

# Rate limit constants for auth endpoints
REGISTER_RATE_LIMIT = "5/minute"  # 5 requests per minute for registration
LOGIN_RATE_LIMIT = "10/minute"    # 10 requests per minute for login

# Rate limit constants for task endpoints (per user)
CREATE_TASK_RATE_LIMIT = "30/hour"   # 30 creates per hour
UPDATE_TASK_RATE_LIMIT = "60/hour"   # 60 updates per hour
DELETE_TASK_RATE_LIMIT = "30/hour"   # 30 deletes per hour
LIST_TASKS_RATE_LIMIT = "100/hour"   # 100 list requests per hour
GET_TASK_RATE_LIMIT = "200/hour"     # 200 get requests per hour
