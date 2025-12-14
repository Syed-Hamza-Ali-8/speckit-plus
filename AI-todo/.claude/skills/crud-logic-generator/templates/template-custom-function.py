def CUSTOM_FUNCTION(id: str, session: Session):
    obj = session.get(MODEL, id)
    if not obj:
        not_found("MODEL not found")

    CUSTOM_LOGIC

    session.add(obj)
    session.commit()
    session.refresh(obj)
    return obj
