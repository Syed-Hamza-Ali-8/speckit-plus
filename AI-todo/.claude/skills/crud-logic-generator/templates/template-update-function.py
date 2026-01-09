def update_MODEL(id: str, data: MODELUpdate, session: Session) -> MODEL:
    obj = session.get(MODEL, id)
    if not obj:
        not_found("MODEL not found")

    for key, value in data.dict(exclude_unset=True).items():
        setattr(obj, key, value)

    session.add(obj)
    session.commit()
    session.refresh(obj)
    return obj
