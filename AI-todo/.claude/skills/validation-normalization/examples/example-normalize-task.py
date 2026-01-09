from templates.template-normalization import normalize_task_payload

data = {"title": "  Finish report  ", "description": "Summary  ", "due_date": "2025-12-10T12:00:00"}
normalized = normalize_task_payload(data)
print(normalized)
