# Plan for RAG Chatbot

## 1. Scope and Dependencies:
- **In Scope:** FastAPI backend, Qdrant for vector search, Neon Postgres for metadata, OpenAI ChatKit for RAG, Document Ingestion pipeline, Dockerization, Deployment instructions.
- **Out of Scope:** Frontend UI (will be a separate project), advanced authentication (basic API key auth), complex NLP models beyond OpenAI ChatKit.
- **External Dependencies:** Qdrant, Neon Postgres, OpenAI API, Docker.

## 2. Key Decisions and Rationale:
- **Backend Framework:** FastAPI - chosen for high performance, ease of use, and strong ecosystem for API development.
- **Vector Database:** Qdrant - selected for its performance, ease of integration, and filtering capabilities.
- **Relational Database:** Neon Postgres - chosen for robust relational data management and scalability.
- **RAG Orchestration:** OpenAI ChatKit - for its capabilities in grounding responses with provided documents.
- **Document Ingestion:** Custom pipeline using Python libraries for text extraction, chunking, and embedding.

## 3. Interfaces and API Contracts:
- **Public APIs:**
    - `/ingest_document`: POST - For uploading and processing documents.
    - `/rag_query`: POST - For querying the RAG system.
    - `/health`: GET - Health check endpoint.

## 4. Non-Functional Requirements (NFRs) and Budgets:
- **Performance:** RAG queries p95 latency < 500ms.
- **Reliability:** 99.9% uptime for core services.
- **Security:** API key authentication for ingestion and query endpoints. Data encryption at rest for sensitive information.

## 5. Data Management and Migration:
- **Source of Truth:** Neon Postgres for document metadata, Qdrant for vector embeddings.
- **Schema Evolution:** Alembic for Postgres schema migrations.
- **Data Retention:** Policy to be defined, likely indefinite for ingested documents.

## 6. Operational Readiness:
- **Observability:** Structured logging, Prometheus/Grafana for metrics, Sentry for error tracking.
- **Alerting:** PagerDuty for critical alerts.
- **Deployment:** Docker containers, Kubernetes for orchestration.

## 7. Risk Analysis and Mitigation:
- **Vendor Lock-in (OpenAI):** Mitigation: Design RAG interface to be modular, allowing for future integration with other LLMs.
- **Data Consistency (Postgres/Qdrant):** Mitigation: Implement transactional updates or eventual consistency patterns with robust error handling and retry mechanisms.
- **Scalability of Ingestion:** Mitigation: Implement a queue-based system for document processing to handle high load.

## 8. Evaluation and Validation:
- **Definition of Done:** All API endpoints implemented and tested, RAG queries return grounded responses, deployment successful.
- **Output Validation:** Input validation on all API endpoints.

## 9. Architectural Decision Record (ADR):
- ADRs will be created for major decisions such as choice of vector database, RAG orchestration framework, and deployment strategy.
