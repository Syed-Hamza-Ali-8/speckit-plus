@router.post("/", response_model=TaskRead)
def create_task(data: TaskCreate, session: Session = Depends(get_session)):
    task = Task(**data.dict())
    session.add(task)
    session.commit()
    session.refresh(task)
    return task
