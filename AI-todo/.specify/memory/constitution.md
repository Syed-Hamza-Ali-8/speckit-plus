# Constitution — The Evolution of Todo – Hackathon II

## Core Philosophy

This constitution governs all development activities for "The Evolution of Todo – Hackathon II" project. The project follows a strict phased approach from a console-based application to a sophisticated event-driven microservices architecture. All AI agents, subagents, skills, and human developers must adhere to these rules without exception.

The project embraces Spec-Driven Development (SDD) as its foundational principle. No code shall be generated, modified, or deployed without a corresponding specification in the `/specs` directory. This ensures predictability, traceability, and alignment between business requirements and technical implementation.

## Spec-Driven Development

1. **Specification Precedence**: All development begins with a spec. No code generation, modification, or deletion occurs without a corresponding spec in `/specs/<feature>/spec.md`.

2. **Spec-Code Synchronization**: Specifications and code must remain synchronized. When code changes require functionality not described in the spec, the spec must be updated first.

3. **Test-Driven Compliance**: All code must include tests that validate against the specification. Tests serve as living documentation of spec compliance.

4. **Spec Approval Process**: Major spec changes require formal approval documented in `/specs/<feature>/approvals/`.

## Monorepo Structure

The project follows a monorepo architecture with strict folder organization:

```
root/
├── .specify/                 # SpecKit Plus configuration
│   └── memory/               # Project constitution
├── specs/                    # All specifications
│   ├── <feature>/           # Feature-specific specs
│   │   ├── spec.md          # Functional requirements
│   │   ├── plan.md          # Architectural decisions
│   │   ├── tasks.md         # Implementation tasks
│   │   └── approvals/       # Change approvals
│   └── architecture.md      # System-wide architecture
├── phase-1/                  # Phase I: Console App
├── phase-2/                  # Phase II: Web Application
├── phase-3/                  # Phase III: MCP Integration
├── phase-4/                  # Phase IV: Kubernetes
├── phase-5/                  # Phase V: Kafka/Dapr
├── history/                  # Historical records
│   ├── prompts/             # Prompt History Records
│   └── adr/                 # Architecture Decision Records
└── .claude/                 # Claude configuration
```

## Phase Rules

### Phase I: Console Application
- **Technology Stack**: Python 3.13, UV package manager
- **Scope**: In-memory console-based todo application
- **Features**: Basic CRUD operations only
- **Constraints**: No database, no API endpoints, no web framework, no agents
- **Data Storage**: In-memory objects only
- **Testing**: Unit tests for all business logic

### Phase II: Web Application
- **Technology Stack**: Next.js 16+ (App Router), FastAPI, SQLModel, Neon DB, Better Auth (JWT)
- **Scope**: Full-stack web application with REST API
- **Features**: Enhanced CRUD with web UI, authentication, database persistence
- **Database**: SQLModel schemas with Neon DB
- **Authentication**: JWT-based with Better Auth integration
- **API**: REST endpoints only, no GraphQL

### Phase III: MCP Integration
- **Technology Stack**: MCP Server (Python SDK), OpenAI Agents SDK
- **Scope**: AI-powered natural language interface
- **Features**: Chat endpoint, MCP tools for CRUD operations, DB-backed conversation history
- **AI Integration**: Natural language processing for todo operations
- **Security**: MCP tool permissions and access controls

### Phase IV: Kubernetes
- **Technology Stack**: Docker (Gordon), Minikube, Helm, kubectl-ai, Kagent
- **Scope**: Containerized deployment with orchestration
- **Features**: AI-assisted DevOps, automated deployments
- **Deployment**: Kubernetes manifests and Helm charts
- **Operations**: kubectl-ai for cluster management

### Phase V: Kafka/Dapr
- **Technology Stack**: Kafka/Redpanda, Dapr (Pub/Sub, Cron Bindings, Secret Store, State Store)
- **Scope**: Event-driven microservices architecture
- **Features**: Distributed event processing, microservices communication
- **Infrastructure**: DigitalOcean Kubernetes
- **Architecture**: Event-driven patterns with Dapr components

## Subagents & Skills

### Phase I Allowed Subagents:
- Spec Agent: Specification creation and validation
- Code Agent: Basic Python code generation
- Refactor Agent: Code restructuring and optimization

### Phase II Allowed Subagents:
- All Phase I agents plus:
- Next.js Agent: Frontend component generation
- FastAPI Agent: Backend API endpoint creation
- Auth/JWT Agent: Authentication and authorization logic

### Phase III Allowed Subagents:
- All Phase II agents plus:
- MCP Tools Agent: MCP server integration
- Agents SDK Agent: OpenAI Agents SDK implementation

### Phase IV Allowed Subagents:
- All Phase III agents plus:
- Docker Agent: Containerization and Dockerfile creation
- Kubernetes Agent: kubectl-ai and kagent operations

### Phase V Allowed Subagents:
- All Phase IV agents plus:
- Kafka Agent: Event streaming and topic management
- Dapr Agent: Dapr component configuration
- Cloud Deployment Agent: DigitalOcean Kubernetes deployment

## Architecture Rules

1. **Clean Architecture**: All phases must follow Clean Architecture principles with clear separation of concerns:
   - Entities (business objects)
   - Use Cases (application business rules)
   - Interface Adapters (data conversion layers)
   - Frameworks & Drivers (external interfaces)

2. **Layer Dependencies**: Dependencies point inward only. Higher-level modules should not depend on lower-level modules.

3. **Interface Contracts**: All inter-layer communication must use well-defined interfaces with strong typing.

4. **Cross-Cutting Concerns**: Logging, monitoring, and error handling are implemented consistently across all layers.

## Authentication Rules

### JWT Validation Requirements:
1. **Token Structure**: JWTs must contain `sub`, `iat`, `exp`, and `scope` claims
2. **Expiration**: Maximum token lifetime of 24 hours for access tokens
3. **Revocation**: Tokens must be validated against database for active status
4. **Scopes**: Permission-based access control using scopes in JWT
5. **Refresh Tokens**: Separate refresh token mechanism with extended validity

### Better Auth Integration:
1. **Provider Configuration**: OAuth providers must be configured in `AUTH_PROVIDERS`
2. **Session Management**: Session persistence follows Better Auth best practices
3. **Password Policies**: Minimum 8 characters, mixed case, numbers, symbols
4. **Rate Limiting**: Account lockout after 5 failed attempts

## MCP Rules

### MCP Server Behavior:
1. **Tool Registration**: All MCP tools must be registered before use
2. **Permission Scoping**: Tools must specify minimal required permissions
3. **Execution Limits**: MCP tools have execution time limits of 30 seconds
4. **Error Handling**: MCP tools must provide structured error responses
5. **Security**: MCP tools run in isolated environments with restricted access

### MCP Tool Development:
1. **Schema Compliance**: All tools must follow JSON Schema specifications
2. **Documentation**: Each tool requires inline documentation and examples
3. **Testing**: MCP tools must include integration tests
4. **Versioning**: MCP tools follow semantic versioning

## Kubernetes Rules

### Deployment Standards:
1. **Manifest Structure**: All deployments use Helm charts with proper templating
2. **Resource Limits**: CPU and memory limits required for all containers
3. **Health Checks**: Liveness and readiness probes mandatory
4. **Secrets Management**: All secrets stored in Kubernetes secrets, never in configmaps
5. **Rolling Updates**: Deployments use rolling update strategy with max surge/Unavailable

### kubectl-ai Usage:
1. **Command Restrictions**: Only approved kubectl commands allowed
2. **Context Validation**: Always validate current cluster context before operations
3. **Dry Run First**: Use dry-run for all complex operations
4. **Audit Trail**: All kubectl operations logged for audit purposes

## Kafka/Dapr Rules

### Kafka Topic Management:
1. **Topic Naming**: Topics follow `<domain>.<entity>.<action>` convention
2. **Partition Strategy**: Partition count based on throughput requirements
3. **Retention Policy**: Configured based on data importance and storage costs
4. **Schema Registry**: All messages use Avro schema with registry integration

### Dapr Component Rules:
1. **Component Types**: Only approved Dapr components (pubsub, state, secret, binding)
2. **Configuration**: Components configured via Kubernetes manifests
3. **Security**: All Dapr communications use mTLS
4. **Observability**: Distributed tracing enabled for all Dapr services

## Code Quality Rules

### Python Standards:
1. **Type Hints**: All functions and methods must include type hints
2. **Async Usage**: Proper async/await patterns in asynchronous code
3. **Error Handling**: Comprehensive exception handling with custom exceptions
4. **Documentation**: All public APIs documented with docstrings

### TypeScript Standards:
1. **Strict Mode**: TypeScript compiler strict mode enabled
2. **Interface Definitions**: Strong typing with interfaces for all data structures
3. **Async Patterns**: Consistent async/await usage
4. **Module Organization**: Logical module structure with barrel exports

### Testing Requirements:
1. **Coverage**: Minimum 80% code coverage for all phases
2. **Types**: Unit, integration, and end-to-end tests as appropriate
3. **Mocking**: Proper mocking of external dependencies
4. **Performance**: Load testing for Phase II and beyond

## Constraints

1. **No Vibe Coding**: All development follows established patterns and specifications
2. **Phase Compliance**: No features from future phases allowed in current phase
3. **Technology Boundaries**: Strict adherence to technology stack for each phase
4. **Architecture Preservation**: No shortcuts that compromise architectural integrity
5. **Dependency Management**: Only approved dependencies allowed

## Success Criteria

### Phase I Success:
- [ ] Basic CRUD operations functional in console
- [ ] All unit tests passing (≥80% coverage)
- [ ] In-memory persistence working correctly
- [ ] Spec-driven development process established

### Phase II Success:
- [ ] Full web application with Next.js frontend
- [ ] FastAPI backend with SQLModel integration
- [ ] Database persistence with Neon DB
- [ ] JWT authentication working end-to-end
- [ ] REST API endpoints documented and tested

### Phase III Success:
- [ ] MCP server operational with tools
- [ ] Natural language chat interface functional
- [ ] AI agents performing CRUD operations
- [ ] Conversation history persisted in database

### Phase IV Success:
- [ ] All services containerized
- [ ] Kubernetes deployment operational
- [ ] Helm charts properly configured
- [ ] AI-assisted DevOps workflows established

### Phase V Success:
- [ ] Event-driven architecture operational
- [ ] Kafka/Redpanda integration complete
- [ ] Dapr components configured
- [ ] Microservices communicating via events

## Enforcement Rules

1. **Spec Verification**: Claude must verify existence of relevant spec before any code generation
2. **Phase Compliance**: Claude must check current phase and reject out-of-phase features
3. **Folder-Level CLAUDE.md**: Claude must read and comply with folder-specific rules
4. **JWT Validation**: All JWT implementations must follow defined validation rules
5. **DB Schema Integrity**: All database changes must maintain referential integrity
6. **API Endpoint Verification**: All API endpoints must match `/specs/api/` definitions

### Rejection Criteria:
- Missing or incomplete specifications
- Requests for future-phase technologies
- Violations of architectural constraints
- Bypassing proper approval processes
- Attempts to skip testing requirements

## Amendment Process

Constitution amendments follow these procedures:

1. **Proposal Location**: Amendments proposed in `/specs/architecture.md` under "Constitution Amendment" section
2. **Approval Process**: Constitutional amendments require explicit approval from project stakeholders
3. **Documentation**: All amendments must be documented with rationale and impact assessment
4. **Version Control**: Amendments are versioned with clear change logs
5. **Communication**: All team members must be notified of constitutional changes
6. **Implementation Timeline**: Amendments take effect after agreed-upon timeline

---

*This constitution is effective immediately upon creation and supersedes all previous development guidelines.*

**Last Updated**: 2025-12-10
**Version**: 1.0.0