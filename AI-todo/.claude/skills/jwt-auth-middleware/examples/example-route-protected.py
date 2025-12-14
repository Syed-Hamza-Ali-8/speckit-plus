@router.get("/me")
async def current_user(user=Depends(verify_jwt)):
    return user
