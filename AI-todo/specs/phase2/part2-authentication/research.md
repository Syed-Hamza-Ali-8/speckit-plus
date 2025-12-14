# Research: Phase 2 Part 2 - Authentication

**Date**: 2025-12-14
**Feature**: User Authentication & Task Ownership

## Research Questions & Findings

### 1. JWT Library Selection

**Question**: Which Python JWT library to use with FastAPI?

**Decision**: `python-jose[cryptography]`

**Rationale**:
- Official FastAPI documentation uses python-jose
- Cryptography backend provides secure implementations
- Well-maintained, widely adopted in production
- Supports HS256 (symmetric) and RS256 (asymmetric)

**Alternatives Considered**:
| Library | Pros | Cons | Status |
|---------|------|------|--------|
| python-jose | FastAPI standard, good docs | Requires cryptography extra | Selected |
| PyJWT | Simple API, popular | Less FastAPI examples | Rejected |
| authlib | Full OAuth support | Overkill for simple JWT | Rejected |

**Configuration**:
```python
# python-jose usage
from jose import jwt, JWTError

token = jwt.encode(payload, SECRET_KEY, algorithm="HS256")
payload = jwt.decode(token, SECRET_KEY, algorithms=["HS256"])
```

---

### 2. Password Hashing Algorithm

**Question**: bcrypt vs Argon2 for password hashing?

**Decision**: Argon2id via `argon2-cffi`

**Rationale**:
- Winner of Password Hashing Competition (2015)
- Memory-hard: resistant to GPU/ASIC attacks
- Argon2id variant: balanced against side-channel and GPU attacks
- OWASP recommendation for new applications
- argon2-cffi provides secure defaults out of the box

**Alternatives Considered**:
| Algorithm | Pros | Cons | Status |
|-----------|------|------|--------|
| Argon2id | Modern, memory-hard, PHC winner | Higher memory usage | Selected |
| bcrypt | Battle-tested, widely deployed | GPU-vulnerable, legacy | Fallback |
| scrypt | Memory-hard | Less tooling, complex tuning | Rejected |
| PBKDF2 | FIPS compliant | Not memory-hard | Rejected |

**Configuration**:
```python
# argon2-cffi usage
from argon2 import PasswordHasher

ph = PasswordHasher()  # Uses secure defaults
hash = ph.hash("password")
ph.verify(hash, "password")  # Raises on mismatch
```

**Default Parameters** (argon2-cffi 23.1.0):
- time_cost: 3 iterations
- memory_cost: 65536 (64 MB)
- parallelism: 4 threads
- hash_len: 32 bytes
- salt_len: 16 bytes

---

### 3. Rate Limiting Implementation

**Question**: How to implement rate limiting on FastAPI endpoints?

**Decision**: `slowapi` with in-memory storage (dev) / Redis (prod)

**Rationale**:
- slowapi is a FastAPI-native rate limiter
- Built on limits library (same as Flask-Limiter)
- Supports decorators per endpoint
- Easy to configure per-IP limiting

**Alternatives Considered**:
| Library | Pros | Cons | Status |
|---------|------|------|--------|
| slowapi | FastAPI native, simple | In-memory not persistent | Selected |
| fastapi-limiter | Async Redis support | Requires Redis always | Rejected |
| Custom middleware | Full control | More code to maintain | Rejected |

**Configuration**:
```python
from slowapi import Limiter
from slowapi.util import get_remote_address

limiter = Limiter(key_func=get_remote_address)

@app.post("/auth/register")
@limiter.limit("5/minute")
async def register(...):
    ...
```

**Production Note**: For horizontal scaling, configure Redis backend:
```python
from slowapi import Limiter
limiter = Limiter(
    key_func=get_remote_address,
    storage_uri="redis://localhost:6379"
)
```

---

### 4. Task Ownership Pattern

**Question**: Middleware-based or service-layer authorization?

**Decision**: Service-layer filtering with dependency injection

**Rationale**:
- More explicit: ownership checks visible in service methods
- More testable: can unit test service methods with mock user_id
- More flexible: supports complex authorization logic later
- Follows FastAPI's dependency injection pattern

**Alternatives Considered**:
| Pattern | Pros | Cons | Status |
|---------|------|------|--------|
| Service-layer filtering | Explicit, testable | More code in services | Selected |
| Middleware | Centralized | Less flexible, harder to test | Rejected |
| Database views | Zero app code | Complex, less portable | Rejected |

**Implementation**:
```python
# Dependency provides current user
async def get_current_user(token: str = Depends(oauth2_scheme)) -> User:
    ...

# Service filters by user_id
class TaskService:
    async def get_tasks(self, db: AsyncSession, user_id: UUID) -> list[Task]:
        result = await db.execute(
            select(Task).where(Task.user_id == user_id)
        )
        return result.scalars().all()
```

---

### 5. Test Database Strategy

**Question**: How to test with both SQLite (fast) and PostgreSQL (realistic)?

**Decision**: Dual-database testing with pytest fixtures

**Rationale**:
- SQLite for fast unit tests (in-memory, no setup)
- Neon PostgreSQL for integration tests (realistic, catches DB-specific issues)
- pytest fixtures provide clean abstraction
- Environment variable controls which DB to use

**Configuration**:
```python
# conftest.py
import pytest
from sqlalchemy.ext.asyncio import create_async_engine

@pytest.fixture
def sqlite_engine():
    """Fast in-memory SQLite for unit tests."""
    return create_async_engine("sqlite+aiosqlite:///:memory:")

@pytest.fixture
def postgres_engine():
    """Neon PostgreSQL for integration tests."""
    return create_async_engine(os.getenv("TEST_DATABASE_URL"))

@pytest.fixture
def db_session(request, sqlite_engine, postgres_engine):
    """Use SQLite by default, Postgres for integration tests."""
    if request.node.get_closest_marker("integration"):
        engine = postgres_engine
    else:
        engine = sqlite_engine
    ...
```

**Usage**:
```python
# Unit test - uses SQLite automatically
async def test_password_hash():
    ...

# Integration test - uses Postgres
@pytest.mark.integration
async def test_auth_flow():
    ...
```

---

### 6. JWT Token Structure

**Question**: What claims to include in JWT?

**Decision**: Minimal claims for MVP

**Claims**:
| Claim | Value | Purpose |
|-------|-------|---------|
| `sub` | User UUID | Subject identifier |
| `email` | User email | Display/logging |
| `exp` | Unix timestamp | Expiration (30 min) |
| `iat` | Unix timestamp | Issued at |

**Deferred to Part 3**:
- `scope` / `permissions` - for role-based access
- `jti` - for token revocation
- `refresh_token` - for extended sessions

**Token Payload Example**:
```json
{
  "sub": "550e8400-e29b-41d4-a716-446655440000",
  "email": "user@example.com",
  "exp": 1734200000,
  "iat": 1734198200
}
```

---

## Dependencies Summary

| Package | Version | Purpose |
|---------|---------|---------|
| python-jose[cryptography] | 3.3.0 | JWT encoding/decoding |
| argon2-cffi | 23.1.0 | Password hashing |
| slowapi | 0.1.9 | Rate limiting |
| aiosqlite | 0.19.0 | SQLite async for tests |

**requirements.txt additions**:
```
python-jose[cryptography]==3.3.0
argon2-cffi==23.1.0
slowapi==0.1.9
aiosqlite==0.19.0
```

---

## Open Questions (Resolved)

| Question | Resolution |
|----------|------------|
| HS256 vs RS256? | HS256 for simplicity (single service) |
| Token expiry? | 30 minutes (balance security/UX) |
| Refresh tokens? | Deferred to Part 3 |
| Email verification? | Deferred to Part 3 |
| Password reset? | Deferred to Part 3 |
