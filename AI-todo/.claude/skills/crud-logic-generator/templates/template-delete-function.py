def delete_MODEL(id: str, session: Session) -> None:
    obj = session.get(MODEL, id)
    if not obj:
        not_found("MODEL not found")

    session.delete(obj)
    session.commit()
