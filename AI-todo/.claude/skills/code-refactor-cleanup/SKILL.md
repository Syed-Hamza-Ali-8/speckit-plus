# SKILL.md — Code Refactor & Cleanup Autopilot Skill
Skill Name: Code Refactor & Cleanup Autopilot
Skill Type: Developer Automation Skill

## Purpose
This skill enables automated code refactoring, formatting, and project cleanup. It helps maintain code quality, modular structure, and readability across Phase 2 workflows while allowing Claude to safely reorganize or split files.

## What This Skill Must Do
1️⃣ Refactor code:
- Extract functions or classes into separate modules
- Rename variables or functions for clarity
- Organize code according to project conventions

2️⃣ Cleanup:
- Remove unused imports or dead code
- Standardize formatting (PEP8 / prettier)
- Consolidate redundant files or modules

3️⃣ Auto-generate templates for refactored files:
- Maintain safe placeholders
- Preserve comments and docstrings

4️⃣ Integrate with version control:
- Suggest changes in branches
- Optionally auto-commit after cleanup

5️⃣ Integrate with other skills:
- CRUD Logic (Skill 12)
- Workflow Engine (Skill 13)
- Validation & Normalization (Skill 18)

## Inputs Expected
- Source code files
- Refactoring instructions
- Target folder / module structure
- Formatting rules
- Optional safety checks

## Outputs of This Skill
- Refactored and formatted source code
- File structure improvements
- Summary of changes (optional diff report)

## Implementation Rules
1. MUST preserve functionality (no breaking changes)
2. MUST maintain spec compliance
3. MUST follow coding standards
4. MUST NOT:
   - Invent new features
   - Modify database schema
   - Change workflows outside safe refactor scope
