def create_MODEL(data: MODELCreate, session: Session) -> MODEL:
    try:
        obj = MODEL(**data.dict())
        session.add(obj)
        session.commit()
        session.refresh(obj)
        return obj
    except Exception:
        conflict("Could not create MODEL")
