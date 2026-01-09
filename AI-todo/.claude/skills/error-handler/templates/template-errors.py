from fastapi import HTTPException

def error_response(error_type: str, message: str, detail=None, status_code: int = 400):
    raise HTTPException(
        status_code=status_code,
        detail={
            "error": {
                "type": error_type,
                "message": message,
                "detail": detail
            }
        }
    )

def not_found(message="Resource not found"):
    return error_response("NotFoundError", message, status_code=404)

def bad_request(message="Invalid request"):
    return error_response("BadRequestError", message, status_code=400)

def unauthorized(message="Unauthorized"):
    return error_response("UnauthorizedError", message, status_code=401)

def forbidden(message="Forbidden"):
    return error_response("ForbiddenError", message, status_code=403)

def conflict(message="Conflict"):
    return error_response("ConflictError", message, status_code=409)
