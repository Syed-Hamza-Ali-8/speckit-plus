# RAG Chatbot Architectural Plan

## 1. Scope and Dependencies
- **In Scope:**
    - Development of a RAG chatbot for answering questions about book content.
    - Context-aware and selective text-based answering.
    - Document ingestion pipeline for PDF/Markdown/HTML to embeddings.
    - FastAPI backend for RAG queries.
    - UI integration for embedding into the book's frontend.
    - Deployment instructions for all components.
    - Speckit task plan for ingestion, updates, and maintenance.
- **Out of Scope:**
    - Detailed book content parsing logic (placeholders will be used).
    - Advanced NLP features beyond RAG.
    - User authentication/authorization for the chatbot itself (unless part of book frontend).
- **External Dependencies:**
    - OpenAI (for embedding model and potentially agents/ChatKit SDK)
    - Qdrant Cloud (Vector Database)
    - Neon Serverless Postgres (Relational Database)

## 2. Key Decisions and Rationale
- **Technical Stack Selection:**
    - **Frontend/SDK**: OpenAI Agents / ChatKit SDK - Chosen for rapid integration and leveraging existing OpenAI ecosystem for chatbot UI.
    - **Backend**: FastAPI - Selected for its high performance, ease of use, and asynchronous capabilities in Python.
    - **Vector Database**: Qdrant Cloud (Free Tier) - Provides efficient vector similarity search and cloud-based deployment.
    - **Relational DB**: Neon Serverless Postgres - Offers a scalable and cost-effective relational database solution for metadata and other structured data.
    - **Embedding model**: OpenAI text-embedding-3-large - State-of-the-art embedding model for accurate semantic search.
- **Modularity and Clean Code:** Emphasis on well-structured, production-ready, and commented code to ensure maintainability and scalability.

## 3. Interfaces and API Contracts
- **FastAPI Backend Endpoints:**
    - `/chat`: Accepts user questions (text, optional selected text) and returns RAG-augmented answers.
    - `/ingest`: Accepts book content (PDF/Markdown/HTML) for processing and embedding.
- **Database Schema (Neon Postgres):**
    - To store metadata about ingested documents, potentially including original text, source, and timestamps.
- **Qdrant Collection:**
    - To store vector embeddings of book content with associated metadata (e.g., chunk ID, source document, page number).

## 4. Non-Functional Requirements (NFRs) and Budgets
- **Performance:**
    - Chatbot response time: p95 latency < 2 seconds.
    - Ingestion pipeline: Efficient processing of documents, scalable for large books.
- **Reliability:**
    - High availability for chatbot service.
    - Robust error handling for API calls and database operations.
- **Security:**
    - Secure handling of API keys (OpenAI, Qdrant, Neon).
    - Data privacy for book content and user interactions.

## 5. Data Management and Migration
- **Source of Truth:** Original book content (PDF/Markdown/HTML) for text, Qdrant for embeddings, Neon Postgres for metadata.
- **Schema Evolution:** Database schemas (Postgres and Qdrant metadata) should be designed for future extensibility.
- **Data Retention:** Policies for retaining original documents, embeddings, and chat history.

## 6. Operational Readiness
- **Observability:**
    - Logging for FastAPI backend (requests, errors, processing steps).
    - Metrics for chatbot usage, response times, and ingestion progress.
- **Alerting:** Thresholds for error rates and performance degradation.
- **Deployment and Rollback:**
    - Containerization (Docker) for FastAPI backend.
    - Clear instructions for deploying Qdrant and Neon Postgres instances.

## 7. Risk Analysis and Mitigation
- **OpenAI API dependency:**
    - Risk: Potential downtime or rate limiting from OpenAI.
    - Mitigation: Implement retry mechanisms, consider fallback strategies (e.g., caching recent answers).
- **Cost management:**
    - Risk: Exceeding free-tier limits for Qdrant and Neon.
    - Mitigation: Monitor usage, optimize embedding storage and query patterns.
- **Book content format variability:**
    - Risk: Difficulty in parsing diverse book formats (PDF, Markdown, HTML).
    - Mitigation: Modularize ingestion pipeline to allow for easy addition of new parsers, use robust parsing libraries.

## 8. Evaluation and Validation
- **Definition of Done:**
    - All requirements met as per `spec.md`.
    - Unit and integration tests for backend.
    - UI integration tested for embedding and functionality.
    - Deployment instructions verified.
    - Speckit task plan generated.
- **Output Validation:** Chatbot answers are accurate, relevant, and consistent with book content.

## 9. Architectural Decision Record (ADR)
- (No ADRs suggested at this stage, as this is the initial plan.)

## Project Structure and Deliverables:

### Output expected:
- Project structure with all files
- `main.py` or equivalent FastAPI backend code
- Frontend integration code
- Database schema for Neon Postgres
- Qdrant setup instructions and connection code
- Speckit task plan

### High-level Project Structure:

```
rag-chatbot/
├── backend/
│   ├── app/
│   │   ├── main.py             # FastAPI application
│   │   ├── api/
│   │   │   ├── chat.py         # Chat endpoint logic
│   │   │   └── ingest.py       # Ingestion endpoint logic
│   │   ├── services/
│   │   │   ├── qdrant_service.py # Qdrant interaction
│   │   │   ├── neon_db_service.py # Neon Postgres interaction
│   │   │   └── embedding_service.py # OpenAI embedding
│   │   ├── models/             # Pydantic models for request/response
│   │   └── core/               # Configuration, utilities
│   ├── ingestion/
│   │   ├── document_parser.py  # Handles PDF/Markdown/HTML parsing
│   │   └── pipeline.py         # Orchestrates ingestion process
│   ├── requirements.txt        # Python dependencies
│   └── Dockerfile              # For containerizing the backend
├── frontend/
│   ├── src/
│   │   ├── components/
│   │   │   └── ChatbotUI.tsx   # React/Vue/Svelte component for chatbot
│   │   ├── sdk/                # OpenAI ChatKit SDK integration
│   │   └── index.html          # Entry point for embedded UI
│   ├── package.json
│   ├── tsconfig.json           # If TypeScript is used
│   └── webpack.config.js       # Or other build tool config
├── database/
│   └── schema.sql              # Neon Postgres schema definition
├── deployment/
│   ├── qdrant_setup.md         # Qdrant setup and connection instructions
│   ├── neon_setup.md           # Neon Postgres setup and connection instructions
│   └── README.md               # Overall deployment instructions
├── docs/
│   └── speckit_tasks.md        # Speckit task plan for maintenance
└── README.md                   # Project overview
```
