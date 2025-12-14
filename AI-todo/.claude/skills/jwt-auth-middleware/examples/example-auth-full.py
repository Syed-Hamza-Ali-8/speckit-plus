from fastapi import Request, HTTPException
from jose import jwt

JWT_SECRET = "secret"
JWT_ALGORITHM = "HS256"

async def verify_jwt(request: Request):
    # Extract
    auth = request.headers.get("Authorization")
    if not auth or not auth.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Missing Authorization header")

    token = auth.split(" ")[1]

    # Decode
    try:
        payload = jwt.decode(token, JWT_SECRET, algorithms=[JWT_ALGORITHM])
    except Exception:
        raise HTTPException(status_code=401, detail="Invalid JWT")

    # Return payload as user dict
    return {"id": payload["id"], "email": payload["email"]}
