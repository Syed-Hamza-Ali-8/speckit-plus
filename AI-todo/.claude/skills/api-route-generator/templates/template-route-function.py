@router.METHOD("ENDPOINT", response_model=RESPONSE_MODEL)
def FUNCTION_NAME(PARAMS, session: Session = Depends(get_session)):
    AUTH_BLOCK
    try:
        CRUD_BODY
    except Exception:
        bad_request("Operation failed")
