from datetime import datetime

def normalize_user_payload(data: dict):
    if "email" in data:
        data["email"] = data["email"].strip().lower()
    if "username" in data:
        data["username"] = data["username"].strip()
    return data

def normalize_task_payload(data: dict):
    if "title" in data:
        data["title"] = data["title"].strip()
    if "description" in data:
        data["description"] = data["description"].strip()
    if "due_date" in data:
        data["due_date"] = datetime.fromisoformat(data["due_date"])
    return data
