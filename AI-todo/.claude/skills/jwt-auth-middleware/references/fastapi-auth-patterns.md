# FastAPI JWT Auth Patterns

## Dependency for authentication:

async def verify_jwt(request: Request):
    token = ...
    payload = decode(token)
    return payload

## Usage in routes:

@router.get("/", dependencies=[Depends(verify_jwt)])
