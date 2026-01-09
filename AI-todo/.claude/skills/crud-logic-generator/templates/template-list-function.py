def list_MODELS(session: Session) -> List[MODEL]:
    try:
        return session.exec(select(MODEL)).all()
    except Exception:
        error_response("DatabaseError", "Failed to list MODELs")
