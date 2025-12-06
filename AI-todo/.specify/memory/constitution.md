<!-- SYNC IMPACT REPORT
Version change: N/A (new constitution) → 1.0.0
Modified principles: N/A
Added sections: Version, Ratification, Principles sections with proper structure
Removed sections: N/A
Templates requiring updates: ⚠ pending - .specify/templates/plan-template.md (constitution check section)
Follow-up TODOs: None
-->

# Project Constitution — Todo In-Memory Python Console App

**Version**: 1.0.0
**Ratification Date**: 2025-12-06
**Last Amended**: 2025-12-06

This constitution defines the governing principles and constraints for all development activities in this project. All team members, including AI assistants, must adhere to these principles.

## Project Identity

This project is a **Todo In-Memory Python Console Application** built using:
- Python 3.13+
- UV package manager
- Claude Code
- Spec-Kit Plus

The primary objective is to create a command-line todo application that stores tasks in memory with basic CRUD functionality.

## Core Principles

### 1. Spec-Driven Development (Mandatory)
All development must follow the spec-driven development workflow:
- Use `/sp.specify` to create feature specifications
- Use `/sp.plan` to generate implementation plans
- Use `/sp.tasks` to create task lists
- Use `/sp.implement` to execute implementation
- All changes must be traceable back to specifications

**Rationale**: Ensures systematic development and prevents scope creep.

### 2. In-Memory Storage Constraint
The application must store all tasks in memory only, with no persistent storage mechanisms:
- No file-based storage
- No database connections
- No external storage services
- Data will be lost when the application terminates

**Rationale**: Maintains simplicity for the initial implementation phase.

### 3. Console Interface Only
The application must provide a command-line interface only:
- No graphical user interface
- No web interface
- No mobile interface
- All interactions through terminal commands

**Rationale**: Focuses on core functionality without UI complexity.

### 4. Basic Todo Operations (Mandatory)
The application must implement exactly 5 core operations:
- **Add**: Create new tasks with title and description
- **View**: List all tasks with status indicators
- **Update**: Modify existing task details
- **Delete**: Remove tasks by ID
- **Mark Complete/Incomplete**: Toggle task completion status

Each task must contain: id, title, description, and completed status.

**Rationale**: Defines the minimum viable product scope.

### 5. Technology Stack Compliance
The project must use the specified technology stack:
- Python 3.13+ as the primary language
- UV for package management
- Claude Code for development assistance
- Spec-Kit Plus for project organization

**Rationale**: Ensures consistency and compatibility.

### 6. Clean Code Standards
All code must follow clean code principles:
- Modular, well-structured code
- Separation of concerns
- Proper function and class organization
- No hardcoded values (use configuration where appropriate)
- Meaningful variable and function names

**Rationale**: Maintains code quality and maintainability.

## Governance

### Amendment Process
This constitution may be amended through:
1. Creation of an Architectural Decision Record (ADR) documenting the change
2. Approval from project stakeholders
3. Update to the constitution file with version increment
4. Propagation of changes to dependent artifacts

### Versioning Policy
- MAJOR version increments for backward incompatible governance/principle changes
- MINOR version increments for new principles or material expansions
- PATCH version increments for clarifications, wording, or non-semantic refinements

### Compliance Review
Regular compliance reviews must verify that all development activities align with these principles. The `/sp.analyze` command should be used to check for consistency across artifacts.

## Scope Boundaries

### In Scope
- Command-line todo application
- In-memory task storage
- Basic CRUD operations
- Task status management
- Proper error handling
- User-friendly console interface

### Out of Scope
- Persistent data storage
- Database integration
- Web or GUI interfaces
- User authentication
- Network connectivity
- Advanced features beyond basic operations

## Repository Structure Requirements

The resulting repository must contain:
- `/src` directory with Python source code
- `/specs` directory with specification files
- `/history/prompts` directory with prompt history records
- `/history/adr` directory with architecture decision records
- `README.md` with setup and usage instructions
- `CLAUDE.md` with Claude Code instructions
- `.specify/` directory with Spec-Kit Plus templates and configurations
