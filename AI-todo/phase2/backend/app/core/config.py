"""Environment configuration using pydantic-settings."""

from functools import lru_cache

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Database
    database_url: str
    test_database_url: str | None = None

    # JWT Configuration
    jwt_secret_key: str
    jwt_algorithm: str = "HS256"
    jwt_access_token_expire_minutes: int = 30

    # OpenAI-compatible API Configuration (works with Gemini, OpenRouter, etc.)
    openai_api_key: str = ""
    openai_base_url: str | None = None
    openai_model: str = "gemini-2.0-flash"
    openai_timeout: int = 30

    # Session Configuration
    session_ttl_minutes: int = 30
    session_max_messages: int = 20

    # Chat Rate Limiting
    chat_rate_limit_per_minute: int = 60

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
    )


@lru_cache
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()  # type: ignore[call-arg]


# For backward compatibility - lazily loaded
settings = get_settings()
