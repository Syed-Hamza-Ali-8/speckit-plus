from fastapi import Request, HTTPException
from jose import jwt
from models.user import UserRead

JWT_SECRET = "YOUR_SECRET"
JWT_ALGORITHM = "HS256"

async def verify_jwt(request: Request) -> UserRead:
    auth = request.headers.get("Authorization")
    if not auth or not auth.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Missing Authorization header")

    token = auth.split(" ")[1]

    try:
        payload = jwt.decode(token, JWT_SECRET, algorithms=[JWT_ALGORITHM])
    except Exception:
        raise HTTPException(status_code=401, detail="Invalid or expired JWT")

    return UserRead(id=payload["id"], email=payload["email"])
