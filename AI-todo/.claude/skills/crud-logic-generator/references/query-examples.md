Filtering:
    select(Task).where(Task.completed == False)

Sorting:
    select(Task).order_by(Task.created_at.desc())
