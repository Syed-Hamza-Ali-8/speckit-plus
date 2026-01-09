# CRUD Patterns

Create:
    obj = Model(**data.dict())
    session.add(obj)
    session.commit()
    session.refresh(obj)
    return obj

Read:
    obj = session.get(Model, id)
    if not obj:
        not_found("Not found")
    return obj

List:
    return session.exec(select(Model)).all()

Update:
    for key, value in data.dict(exclude_unset=True).items():
        setattr(obj, key, value)
    session.add(obj)
    session.commit()
    session.refresh(obj)
    return obj

Delete:
    session.delete(obj)
    session.commit()
