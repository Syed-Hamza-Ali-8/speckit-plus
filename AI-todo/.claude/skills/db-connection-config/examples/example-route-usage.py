@router.get("/")
def list_tasks(session: Session = Depends(get_session)):
    return session.exec(select(Task)).all()
