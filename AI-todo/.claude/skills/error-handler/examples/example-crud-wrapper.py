def get_task(id: str, session: Session):
    try:
        task = session.get(Task, id)
        if not task:
            not_found("Task does not exist")
        return task
    except IntegrityError:
        conflict("Invalid task ID")
