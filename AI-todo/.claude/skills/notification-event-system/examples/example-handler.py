def on_task_created(data):
    print("Task created:", data)

event_bus.on("task.created", on_task_created)
