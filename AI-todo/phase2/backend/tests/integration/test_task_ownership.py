"""Integration tests for task ownership and isolation."""

from uuid import uuid4

import pytest
from httpx import AsyncClient


@pytest.fixture
async def user_a_with_task(test_client: AsyncClient) -> dict:
    """Create User A with a task and return their data."""
    email = f"user_a_{uuid4().hex[:8]}@example.com"
    password = "password123"

    # Register User A
    reg_response = await test_client.post(
        "/auth/register",
        json={"email": email, "password": password},
    )
    assert reg_response.status_code == 201
    user_data = reg_response.json()

    # Login User A
    login_response = await test_client.post(
        "/auth/login",
        json={"email": email, "password": password},
    )
    assert login_response.status_code == 200
    token = login_response.json()["access_token"]
    headers = {"Authorization": f"Bearer {token}"}

    # Create a task for User A
    task_response = await test_client.post(
        "/tasks",
        json={"title": "User A's Task", "description": "Private to User A"},
        headers=headers,
    )
    assert task_response.status_code == 201
    task_data = task_response.json()

    return {
        "user_id": user_data["id"],
        "email": email,
        "token": token,
        "headers": headers,
        "task_id": task_data["id"],
        "task": task_data,
    }


@pytest.fixture
async def user_b_credentials(test_client: AsyncClient) -> dict:
    """Create User B and return their credentials."""
    email = f"user_b_{uuid4().hex[:8]}@example.com"
    password = "password123"

    # Register User B
    reg_response = await test_client.post(
        "/auth/register",
        json={"email": email, "password": password},
    )
    assert reg_response.status_code == 201
    user_data = reg_response.json()

    # Login User B
    login_response = await test_client.post(
        "/auth/login",
        json={"email": email, "password": password},
    )
    assert login_response.status_code == 200
    token = login_response.json()["access_token"]

    return {
        "user_id": user_data["id"],
        "email": email,
        "token": token,
        "headers": {"Authorization": f"Bearer {token}"},
    }


class TestTaskOwnership:
    """T051-T052: Test task isolation between users."""

    @pytest.mark.asyncio
    async def test_user_lists_only_own_tasks(
        self, test_client: AsyncClient, user_a_with_task: dict, user_b_credentials: dict
    ) -> None:
        """TC-15: User A lists only User A's tasks, not User B's."""
        # Create a task for User B
        task_b_response = await test_client.post(
            "/tasks",
            json={"title": "User B's Task"},
            headers=user_b_credentials["headers"],
        )
        assert task_b_response.status_code == 201

        # User A lists their tasks
        user_a_tasks = await test_client.get(
            "/tasks",
            headers=user_a_with_task["headers"],
        )
        assert user_a_tasks.status_code == 200
        a_response = user_a_tasks.json()
        # Handle both paginated {"items": [...]} and direct list responses
        a_items = a_response.get("items", a_response) if isinstance(a_response, dict) else a_response
        a_task_ids = [t["id"] for t in a_items]

        # User B lists their tasks
        user_b_tasks = await test_client.get(
            "/tasks",
            headers=user_b_credentials["headers"],
        )
        assert user_b_tasks.status_code == 200
        b_response = user_b_tasks.json()
        # Handle both paginated {"items": [...]} and direct list responses
        b_items = b_response.get("items", b_response) if isinstance(b_response, dict) else b_response
        b_task_ids = [t["id"] for t in b_items]

        # User A should see their task but not User B's
        assert user_a_with_task["task_id"] in a_task_ids
        assert task_b_response.json()["id"] not in a_task_ids

        # User B should see their task but not User A's
        assert task_b_response.json()["id"] in b_task_ids
        assert user_a_with_task["task_id"] not in b_task_ids

    @pytest.mark.asyncio
    async def test_user_cannot_get_others_task(
        self, test_client: AsyncClient, user_a_with_task: dict, user_b_credentials: dict
    ) -> None:
        """TC-16: User B trying to get User A's task returns 404."""
        # User B tries to get User A's task
        response = await test_client.get(
            f"/tasks/{user_a_with_task['task_id']}",
            headers=user_b_credentials["headers"],
        )

        assert response.status_code == 404
        assert "not found" in response.json()["detail"].lower()

    @pytest.mark.asyncio
    async def test_user_cannot_update_others_task(
        self, test_client: AsyncClient, user_a_with_task: dict, user_b_credentials: dict
    ) -> None:
        """User B cannot update User A's task."""
        response = await test_client.patch(
            f"/tasks/{user_a_with_task['task_id']}",
            json={"title": "Hacked Title"},
            headers=user_b_credentials["headers"],
        )

        assert response.status_code == 404

        # Verify task is unchanged
        original_task = await test_client.get(
            f"/tasks/{user_a_with_task['task_id']}",
            headers=user_a_with_task["headers"],
        )
        assert original_task.json()["title"] == "User A's Task"

    @pytest.mark.asyncio
    async def test_user_cannot_delete_others_task(
        self, test_client: AsyncClient, user_a_with_task: dict, user_b_credentials: dict
    ) -> None:
        """User B cannot delete User A's task."""
        response = await test_client.delete(
            f"/tasks/{user_a_with_task['task_id']}",
            headers=user_b_credentials["headers"],
        )

        assert response.status_code == 404

        # Verify task still exists for User A
        task_response = await test_client.get(
            f"/tasks/{user_a_with_task['task_id']}",
            headers=user_a_with_task["headers"],
        )
        assert task_response.status_code == 200


class TestTaskCRUD:
    """Additional task endpoint tests."""

    @pytest.mark.asyncio
    async def test_create_task_requires_auth(self, test_client: AsyncClient) -> None:
        """Creating a task without auth returns 401."""
        response = await test_client.post(
            "/tasks",
            json={"title": "Unauthorized Task"},
        )

        assert response.status_code == 401

    @pytest.mark.asyncio
    async def test_create_and_complete_task(
        self, test_client: AsyncClient, auth_headers: dict
    ) -> None:
        """Full task lifecycle: create, update status, delete."""
        # Create
        create_response = await test_client.post(
            "/tasks",
            json={"title": "Lifecycle Task", "description": "Test lifecycle"},
            headers=auth_headers,
        )
        assert create_response.status_code == 201
        task = create_response.json()
        assert task["status"] == "pending"

        # Update to completed
        update_response = await test_client.patch(
            f"/tasks/{task['id']}",
            json={"status": "completed"},
            headers=auth_headers,
        )
        assert update_response.status_code == 200
        assert update_response.json()["status"] == "completed"

        # Delete
        delete_response = await test_client.delete(
            f"/tasks/{task['id']}",
            headers=auth_headers,
        )
        assert delete_response.status_code == 204

        # Verify deleted
        get_response = await test_client.get(
            f"/tasks/{task['id']}",
            headers=auth_headers,
        )
        assert get_response.status_code == 404
