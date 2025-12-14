@router.patch("/{id}", response_model=TaskRead)
def update_task(id: str, data: TaskUpdate, session: Session = Depends(get_session)):
    task = session.get(Task, id)
    if not task:
        not_found("Task not found")
    for key, value in data.dict(exclude_unset=True).items():
        setattr(task, key, value)
    session.add(task)
    session.commit()
    session.refresh(task)
    return task
