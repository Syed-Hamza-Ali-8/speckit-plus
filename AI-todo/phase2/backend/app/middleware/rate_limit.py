"""Rate limiting configuration using slowapi."""

from slowapi import Limiter
from slowapi.util import get_remote_address

# Create limiter instance using client IP for rate limit key
limiter = Limiter(key_func=get_remote_address)

# Rate limit constants
REGISTER_RATE_LIMIT = "5/minute"  # 5 requests per minute for registration
LOGIN_RATE_LIMIT = "10/minute"    # 10 requests per minute for login
