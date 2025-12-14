# SKILL.md — Validation & Request Normalization Skill
Skill Name: Validation & Request Normalization
Skill Type: Backend Infrastructure Skill

## Purpose
This skill ensures that all incoming data (requests, API inputs, workflow payloads) are validated, sanitized, and normalized before processing. It provides consistent, safe, and spec-compliant input handling across Phase 2.

## What This Skill Must Do
1️⃣ Validate inputs:
- Ensure required fields exist
- Validate types (string, int, list, etc.)
- Check formats (email, date, regex)

2️⃣ Normalize data:
- Trim strings
- Standardize date/time formats
- Convert numeric types
- Apply default values if missing

3️⃣ Sanitize inputs:
- Remove dangerous characters
- Prevent injection attacks
- Clean user-provided data

4️⃣ Integrate with other skills:
- CRUD Logic (Skill 12)
- Workflow Engine (Skill 13)
- Async Tasks (Skill 15)
- Notification System (Skill 14)

5️⃣ Templates for common validations:
- Task creation payload
- User registration payload
- Workflow action input

## Inputs Expected
- Raw request or payload data
- Validation rules (from spec)
- Optional default values
- Optional normalization rules

## Outputs of This Skill
- Validated and normalized data object
- Validation error messages
- Safe payload ready for processing

## Implementation Rules
1. MUST use Pydantic or equivalent for validation
2. MUST raise clear errors for invalid inputs
3. MUST provide reusable validation schemas
4. MUST NOT:
   - Modify CRUD logic
   - Modify database schema
   - Invent fields not in spec
