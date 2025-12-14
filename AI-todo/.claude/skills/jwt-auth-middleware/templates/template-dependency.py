from fastapi import Depends

@router.get("/secure", response_model=UserRead)
async def secure_route(user=Depends(verify_jwt)):
    return user
