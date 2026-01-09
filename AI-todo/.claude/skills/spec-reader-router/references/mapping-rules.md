# Subagent Mapping Rules

## UI Pages → nextjs-page-generator
/specs/ui/pages/*.md
frontend/app/**/*.tsx

## UI Components → tailwind-ui-component-builder
/specs/ui/components/*.md
frontend/components/*.tsx

## API Routes → fastapi-crud-generator
/specs/api/*.md
backend/routes/*.py

## JWT Auth → jwt-auth-middleware
/specs/api/auth/*.md
backend/utils/auth.py

## DB Schema → sqlmodel-schema-generator
/specs/database/*.md
backend/models/*.py

## DB Engine/Session → db-connection-config
backend/db/session.py
