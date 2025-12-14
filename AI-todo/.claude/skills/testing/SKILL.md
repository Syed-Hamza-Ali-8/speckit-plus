# SKILL.md — Testing Skill
Skill Name: Testing
Skill Type: Developer Automation Skill

## Purpose
This skill automates the generation of tests for Phase 2 workflows, ensuring correctness, reliability, and regression prevention. It supports unit tests, integration tests, and API endpoint tests.

## What This Skill Must Do
1️⃣ Generate unit tests:
- For models, CRUD operations, and utility functions
- Use pytest conventions
- Include edge cases and validation errors

2️⃣ Generate integration tests:
- Combine multiple modules
- Test workflows end-to-end
- Ensure async tasks and events are executed correctly

3️⃣ Generate API tests:
- Test API endpoints (GET, POST, PUT, DELETE)
- Validate request/response schemas
- Check status codes and error handling

4️⃣ Integrate with CI/CD:
- Generate ready-to-run test files
- Include fixtures for database and session

5️⃣ Templates for common testing patterns:
- Unit test for a model
- Integration test for workflow
- API test for endpoint

## Inputs Expected
- Target module / file
- Testing rules or spec
- Sample data / fixtures
- Optional coverage targets

## Outputs of This Skill
- pytest-compatible test files
- Example fixtures
- Summary of test results (optional)
- Optional code coverage report

## Implementation Rules
1. MUST use pytest
2. MUST provide reusable fixtures
3. MUST NOT:
   - Modify production code
   - Invent tests outside spec
   - Touch frontend directly
