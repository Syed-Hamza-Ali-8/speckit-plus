def list_tasks(session):
    return session.exec(select(Task)).all()
