Skill Name: Database Session & Connection Config Skill
Skill Type: Backend Database Infrastructure Skill
Purpose

This skill generates and maintains all logic required to:

Connect FastAPI backend to Neon PostgreSQL

Create SQLModel database engine

Create session dependency for CRUD operations

Manage environment variables for DB_URL

Handle connection pooling & async/regular engines based on spec

Ensure correct integration with API routes

This skill ensures your backend always has a reliable DB connection following Phase-2 requirements.

What This Skill Must Do

Generate engine configuration using SQLModel

Create get_session() dependency

Pull DB credentials from environment

Ensure correct Neon-compatible connection string

Create a db/session.py file with proper setup

Support both sync and async models if spec requires

Inputs Expected

User may provide:

Location for DB file
backend/db/session.py

Environment variable name
DATABASE_URL

Neon connection string example

Session usage requirements in routes

Output of This Skill

A complete database connection module:

backend/db/session.py


Containing:

engine

session maker

get_session() dependency

environment loading logic

Implementation Rules
1. MUST load env from settings
import os
DATABASE_URL = os.getenv("DATABASE_URL")

2. MUST create SQLModel engine
engine = create_engine(DATABASE_URL, echo=False)


or async version:

engine = create_async_engine(DATABASE_URL.replace("postgres://", "postgresql+asyncpg://"))

3. MUST create session dependency
def get_session():
    with Session(engine) as session:
        yield session

4. MUST work with Neon PostgreSQL

Neon requires:

pooled connections

SSL enabled

correct URL scheme:
postgresql://user:pass@host/db?sslmode=require

5. MUST NOT create migrations

This is handled externally.

6. MUST NOT create models or routes

This skill is infrastructure only.

Example Usage
Example 1
Use skill: db-connection-config
Generate: backend/db/session.py

Example 2
Use skill: db-connection-config
Update engine for Neon SSL requirement

Constraints

MUST NOT modify models

MUST NOT modify CRUD routes

MUST NOT add business logic

ONLY infrastructure config