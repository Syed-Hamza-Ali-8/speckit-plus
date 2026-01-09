# SQLModel Database Patterns

## Create
session.add(obj)
session.commit()
session.refresh(obj)

## Select
session.exec(select(Task).where(Task.id == id))

## Update
for key, value in data.items():
    setattr(task, key, value)
session.add(task)
session.commit()
