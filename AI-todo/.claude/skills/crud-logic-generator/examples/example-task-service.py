def mark_complete(id: str, session: Session):
    task = session.get(Task, id)
    if not task:
        not_found("Task not found")
    task.completed = True
    session.add(task)
    session.commit()
    session.refresh(task)
    return task
