"""Integration tests for chat endpoint.

Tests the POST /chat endpoint with various intent scenarios
including task operations and confirmation flows.
"""

import json
from unittest.mock import AsyncMock, patch
from uuid import uuid4

import pytest
from httpx import AsyncClient


class TestChatEndpoint:
    """Tests for POST /chat endpoint."""

    @pytest.mark.asyncio
    async def test_chat_requires_auth(self, test_client: AsyncClient) -> None:
        """Test that chat endpoint requires authentication."""
        response = await test_client.post(
            "/chat",
            json={"message": "Show my tasks"},
        )
        assert response.status_code == 401

    @pytest.mark.asyncio
    async def test_chat_greeting(
        self,
        test_client: AsyncClient,
        auth_headers: dict[str, str],
    ) -> None:
        """Test chat handles greeting messages."""
        # Mock the OpenAI call to avoid actual API calls
        with patch(
            "app.agents.openai_client.chat_completion",
            new_callable=AsyncMock,
        ) as mock_openai:
            mock_openai.return_value = json.dumps({
                "intent": "chat",
                "confidence": 0.95,
            })

            response = await test_client.post(
                "/chat",
                json={"message": "Hello!"},
                headers=auth_headers,
            )

            assert response.status_code == 200
            data = response.json()
            assert "response" in data
            assert "session_id" in data
            assert data["intent"] == "chat"

    @pytest.mark.asyncio
    async def test_chat_list_tasks(
        self,
        test_client: AsyncClient,
        auth_headers: dict[str, str],
    ) -> None:
        """Test chat handles read intent to list tasks."""
        with patch(
            "app.agents.openai_client.chat_completion",
            new_callable=AsyncMock,
        ) as mock_openai:
            mock_openai.return_value = json.dumps({
                "intent": "read",
                "confidence": 0.92,
            })

            response = await test_client.post(
                "/chat",
                json={"message": "Show my tasks"},
                headers=auth_headers,
            )

            assert response.status_code == 200
            data = response.json()
            assert data["intent"] == "read"
            # Should have list_tasks action
            actions = data.get("actions", [])
            if actions:
                assert any(a["tool"] == "list_tasks" for a in actions)

    @pytest.mark.asyncio
    async def test_chat_create_task(
        self,
        test_client: AsyncClient,
        auth_headers: dict[str, str],
    ) -> None:
        """Test chat handles create intent."""
        with patch(
            "app.agents.openai_client.chat_completion",
            new_callable=AsyncMock,
        ) as mock_openai:
            mock_openai.return_value = json.dumps({
                "intent": "create",
                "confidence": 0.88,
                "task_title": "Buy groceries",
            })

            response = await test_client.post(
                "/chat",
                json={"message": "Add a task to buy groceries"},
                headers=auth_headers,
            )

            assert response.status_code == 200
            data = response.json()
            assert data["intent"] == "create"

    @pytest.mark.asyncio
    async def test_chat_session_continuity(
        self,
        test_client: AsyncClient,
        auth_headers: dict[str, str],
    ) -> None:
        """Test that session_id persists across requests."""
        with patch(
            "app.agents.openai_client.chat_completion",
            new_callable=AsyncMock,
        ) as mock_openai:
            mock_openai.return_value = json.dumps({
                "intent": "chat",
                "confidence": 0.9,
            })

            # First request
            response1 = await test_client.post(
                "/chat",
                json={"message": "Hello"},
                headers=auth_headers,
            )
            assert response1.status_code == 200
            session_id = response1.json()["session_id"]
            assert session_id is not None

            # Second request with same session
            response2 = await test_client.post(
                "/chat",
                json={"message": "Hello again", "session_id": session_id},
                headers=auth_headers,
            )
            assert response2.status_code == 200
            assert response2.json()["session_id"] == session_id

    @pytest.mark.asyncio
    async def test_chat_message_validation(
        self,
        test_client: AsyncClient,
        auth_headers: dict[str, str],
    ) -> None:
        """Test message validation (empty, too long)."""
        # Empty message
        response = await test_client.post(
            "/chat",
            json={"message": ""},
            headers=auth_headers,
        )
        assert response.status_code == 422

        # Too long message (>2000 chars)
        response = await test_client.post(
            "/chat",
            json={"message": "x" * 2001},
            headers=auth_headers,
        )
        assert response.status_code == 422

    @pytest.mark.asyncio
    async def test_chat_response_metadata(
        self,
        test_client: AsyncClient,
        auth_headers: dict[str, str],
    ) -> None:
        """Test response includes required metadata."""
        with patch(
            "app.agents.openai_client.chat_completion",
            new_callable=AsyncMock,
        ) as mock_openai:
            mock_openai.return_value = json.dumps({
                "intent": "chat",
                "confidence": 0.9,
            })

            response = await test_client.post(
                "/chat",
                json={"message": "Hi there"},
                headers=auth_headers,
            )

            assert response.status_code == 200
            data = response.json()

            # Check metadata
            assert "metadata" in data
            metadata = data["metadata"]
            assert "processing_time_ms" in metadata
            assert "agent_chain" in metadata
            assert "model" in metadata
            assert isinstance(metadata["agent_chain"], list)
            assert metadata["processing_time_ms"] >= 0


class TestChatSession:
    """Tests for session management."""

    @pytest.mark.asyncio
    async def test_invalid_session_creates_new(
        self,
        test_client: AsyncClient,
        auth_headers: dict[str, str],
    ) -> None:
        """Test that invalid session_id creates a new session."""
        with patch(
            "app.agents.openai_client.chat_completion",
            new_callable=AsyncMock,
        ) as mock_openai:
            mock_openai.return_value = json.dumps({
                "intent": "chat",
                "confidence": 0.9,
            })

            # Send with non-existent session_id
            fake_session_id = str(uuid4())
            response = await test_client.post(
                "/chat",
                json={"message": "Hello", "session_id": fake_session_id},
                headers=auth_headers,
            )

            assert response.status_code == 200
            # Should get a new session_id
            data = response.json()
            assert data["session_id"] != fake_session_id


class TestDeleteConfirmation:
    """Tests for delete confirmation flow."""

    @pytest.mark.asyncio
    async def test_delete_requires_confirmation(
        self,
        test_client: AsyncClient,
        auth_headers: dict[str, str],
    ) -> None:
        """Test that delete intent triggers confirmation prompt."""
        with patch(
            "app.agents.openai_client.chat_completion",
            new_callable=AsyncMock,
        ) as mock_openai:
            mock_openai.return_value = json.dumps({
                "intent": "delete",
                "confidence": 0.85,
            })

            response = await test_client.post(
                "/chat",
                json={"message": "Delete that task"},
                headers=auth_headers,
            )

            assert response.status_code == 200
            data = response.json()
            # Response should ask for confirmation
            assert "confirm" in data["response"].lower() or "sure" in data["response"].lower()
