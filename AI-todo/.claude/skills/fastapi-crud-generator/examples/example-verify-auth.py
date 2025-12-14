user = verify_jwt(request)
if not user:
    raise HTTPException(status_code=401, detail="Invalid token")
