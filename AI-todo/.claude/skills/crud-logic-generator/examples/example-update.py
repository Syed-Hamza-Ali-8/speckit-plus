def update_task(id, data, session):
    task = session.get(Task, id)
    if not task:
        not_found("Task not found")
    for key, value in data.dict(exclude_unset=True).items():
        setattr(task, key, value)
    session.commit()
    return task
