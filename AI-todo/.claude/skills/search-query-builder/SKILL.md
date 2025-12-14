# SKILL.md — Search & Query Builder Skill
Skill Name: Search & Query Builder
Skill Type: Backend Infrastructure Skill

## Purpose
This skill provides advanced search and query capabilities for Phase 2 workflows. It allows multi-criteria filtering, pagination, sorting, and dynamic query generation for CRUD operations and reports.

## What This Skill Must Do
1️⃣ Build dynamic queries:
- Filter by multiple fields
- Support operators: equals, contains, in, range, etc.
- Combine conditions with AND / OR

2️⃣ Support pagination:
- Limit / offset
- Page numbers
- Total counts

3️⃣ Support sorting:
- By one or multiple fields
- Ascending / descending

4️⃣ Integrate with other skills:
- Works with CRUD (Skill 11 / 12)
- Activity Log (Skill 16)
- Workflow Engine (Skill 13)

5️⃣ Templates for common query patterns:
- Search by user
- Search by task status
- Search by date ranges

## Inputs Expected
- Search criteria (dict or object)
- Pagination settings
- Sorting instructions
- Optional filters defined in spec

## Outputs of This Skill
- Query result sets
- Metadata (pagination info)
- Filtered and sorted lists of entities

## Implementation Rules
1. MUST generate safe queries (avoid SQL injection)
2. MUST support ORM (SQLModel / SQLAlchemy)
3. MUST provide reusable query builder templates
4. MUST NOT:
   - Modify CRUD logic
   - Handle frontend UI
   - Invent filters not defined in spec
