from fastapi.testclient import TestClient
from backend.main import app

client = TestClient(app)

def test_get_tasks():
    response = client.get("/tasks/")
    assert response.status_code == 200
    assert isinstance(response.json(), list)

def test_create_task():
    payload = {"title": "API Task", "assigned_to": "user_123"}
    response = client.post("/tasks/", json=payload)
    assert response.status_code == 200
    data = response.json()
    assert data["title"] == "API Task"
    assert data["assigned_to"] == "user_123"
