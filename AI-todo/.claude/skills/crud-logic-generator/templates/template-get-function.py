def get_MODEL(id: str, session: Session) -> MODEL:
    obj = session.get(MODEL, id)
    if not obj:
        not_found("MODEL not found")
    return obj
