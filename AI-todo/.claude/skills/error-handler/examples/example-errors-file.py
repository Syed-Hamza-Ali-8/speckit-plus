from fastapi import HTTPException

def not_found(msg="Not found"):
    raise HTTPException(status_code=404, detail={"error": {"type": "NotFoundError", "message": msg}})
