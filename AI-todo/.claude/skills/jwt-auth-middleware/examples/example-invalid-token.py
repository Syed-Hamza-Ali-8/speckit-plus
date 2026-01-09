if not token:
    raise HTTPException(status_code=401, detail="Authorization token missing")
