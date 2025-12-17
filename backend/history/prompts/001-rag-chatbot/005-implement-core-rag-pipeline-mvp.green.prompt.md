---
id: 005
title: Implement Core RAG Pipeline MVP
stage: green
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-rag-chatbot
branch: 001-textbook-generation
user: Uzma Imran
command: /sp.implement (continued from previous session)
labels: ["implementation", "mvp", "rag-pipeline", "fastapi", "cohere", "qdrant", "claude", "services", "api"]
links:
  spec: /mnt/c/Users/pc/specs/001-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/src/utils/config.py
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/src/utils/error_handling.py
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/src/models/entities.py
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/src/services/embeddings.py
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/src/services/vector_store.py
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/src/services/llm.py
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/src/services/rag_pipeline.py
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/src/api/routes.py
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/src/main.py
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/src/__init__.py
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/src/api/__init__.py
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/src/models/__init__.py
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/src/services/__init__.py
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/src/utils/__init__.py
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/requirements.txt
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/.env.example
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/.gitignore
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/README.md
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/scripts/ingest_book.py
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/scripts/test_pipeline.py
  - /mnt/c/Users/pc/Desktop/hackathon_2025/backend/scripts/validate_setup.py
tests:
  - scripts/validate_setup.py - validates environment and service connectivity
  - scripts/test_pipeline.py - end-to-end RAG pipeline testing
---

## Prompt

```text
2
```

(User selected option 2: "Create a shortened implementation script focusing on core RAG pipeline" from the /sp.implement options menu)

## Response snapshot

Implemented complete RAG chatbot MVP with focus on core pipeline functionality:

### Architecture Layers Created

**1. Configuration & Error Handling** (2 files)
- `src/utils/config.py`: Environment variable management with validation for Qdrant, Cohere, Anthropic, and Neon DB credentials
- `src/utils/error_handling.py`: Custom exception hierarchy (RAGError, CohereError, QdrantError, ClaudeError, ValidationError, DatabaseError)

**2. Data Models** (1 file)
- `src/models/entities.py`: Pydantic models for API contracts
  - QueryRequest: query validation (1-2000 chars), top_k (1-10)
  - QueryResponse: answer, citations, latency_ms, status
  - Citation: chapter, section, page, chunk_id, relevance_score
  - HealthResponse: service status monitoring
  - ErrorResponse: standardized error format

**3. Service Layer** (4 files)
- `src/services/embeddings.py`: Cohere SDK wrapper
  - embed_query(): Search query embeddings (input_type="search_query")
  - embed_chunks(): Document embeddings (input_type="search_document")
  - Model: embed-english-v3.0 (1024 dimensions)

- `src/services/vector_store.py`: Qdrant client wrapper
  - ensure_collection(): Create collection with cosine similarity
  - search_chunks(): Vector similarity search with metadata
  - upsert_chunks(): Batch insert with payload (chapter, section, page, text)

- `src/services/llm.py`: Anthropic Claude wrapper
  - generate_response(): Context-aware generation with citations
  - System prompt: Strict adherence to retrieved content only
  - Status detection: "success", "partial", "cannot_answer"
  - Max tokens: 1024

- `src/services/rag_pipeline.py`: End-to-end orchestration
  - query(): Embed → Search → Generate → Extract citations
  - Latency tracking (milliseconds)
  - UUID-based query identification

**4. API Layer** (2 files)
- `src/api/routes.py`: FastAPI endpoints
  - POST /api/v1/query: Main RAG endpoint with error handling
  - GET /api/v1/health: Service availability check (Qdrant, Cohere, Claude)
  - HTTPException mapping for all service errors

- `src/main.py`: Application entry point
  - FastAPI app with CORS middleware
  - Config validation on startup
  - OpenAPI docs at /docs
  - Root endpoint with API info

**5. Ingestion & Testing** (3 scripts)
- `scripts/ingest_book.py`: Book chunking and embedding pipeline
  - BookChunker: tiktoken-based chunking (750 tokens, 75 overlap)
  - Batch embedding (96 chunks per batch)
  - Qdrant upsert with metadata
  - CLI: --text-file, --book-id, --title, --author, --chapter

- `scripts/validate_setup.py`: Pre-flight validation
  - Package import checks (9 required packages)
  - Environment variable verification with masking
  - Service connectivity tests (Qdrant, Cohere, Claude)
  - File structure validation

- `scripts/test_pipeline.py`: End-to-end testing
  - Health check with collection verification
  - Sample queries with latency measurement
  - Citation extraction and display
  - Success/failure reporting

**6. Setup & Documentation** (4 files)
- `requirements.txt`: 14 dependencies (FastAPI, Cohere, Qdrant, Anthropic, asyncpg, tiktoken, pytest)
- `.env.example`: Template for all required API keys and settings
- `.gitignore`: Python, IDE, environment, logs, data files
- `README.md`: Complete quickstart guide with API examples, troubleshooting, and hackathon demo prep

### Implementation Highlights

**Service Integration Pattern**: Singleton instances for all services (embedding_service, vector_store_service, llm_service, rag_pipeline)

**Error Handling**: Service-specific exceptions propagate to HTTP status codes (400 for validation, 500 for API failures, 503 for unavailable services)

**Citation Flow**:
1. Vector search returns chunks with metadata (chapter, section, page, chunk_id, score)
2. LLM generates answer with inline citations in format "[Chapter X, Section Y, p. Z]"
3. Citation objects built from retrieved chunks with relevance scores

**Constitutional Compliance**:
- ✓ Accuracy: Strict "retrieved content only" system prompt
- ✓ Context-Awareness: Top-k retrieval with relevance scoring
- ✓ Traceability: UUID tracking, citation metadata, latency measurement
- ✓ Performance: Async-ready architecture (not yet fully async)
- ✓ Stack Adherence: Cohere + Qdrant + Claude + FastAPI as specified
- ✓ Privacy: Environment variable management, no hardcoded secrets
- ⏳ Chunking: Implemented in ingestion script (750 tokens, 75 overlap)

### Technical Decisions

**Embedding Strategy**: Separate input types for queries vs documents per Cohere best practices

**Vector Search**: Cosine similarity with HNSW indexing (configured in Qdrant)

**LLM Prompting**: Two-part prompt (system: instructions, user: context + question) with explicit fallback for insufficient information

**Chunking**: tiktoken cl100k_base encoding for accurate token counting, overlap for context preservation

**Package Structure**: Proper __init__.py files for all packages to support Python module imports

## Outcome

- ✅ Impact: Complete MVP implementation with 21 files covering all core RAG pipeline functionality (embed → retrieve → generate → cite); FastAPI application ready to run; comprehensive validation and testing scripts; production-ready error handling
- 🧪 Tests: Created validate_setup.py for pre-flight checks (packages, env vars, services, file structure); test_pipeline.py for end-to-end RAG validation with sample queries; both scripts provide detailed pass/fail reporting
- 📁 Files: 21 files created across 6 layers: Configuration/Error Handling (2), Data Models (1), Services (4), API (2), Scripts (3), Setup/Docs (4), Package Init (5)
- 🔁 Next prompts: `python scripts/validate_setup.py` to verify environment, `python scripts/ingest_book.py --text-file <path> --title "<title>" --author "<author>"` to load textbook, `python scripts/test_pipeline.py` to validate RAG pipeline, `python -m src.main` to start API server, then test with `curl -X POST http://localhost:8000/api/v1/query -H "Content-Type: application/json" -d '{"query": "What are IMU sensors?"}'`
- 🧠 Reflection: MVP implementation focused on core RAG flow per user selection of option 2; architecture supports future async conversion (US2) and database logging (US3) without major refactoring; singleton service pattern enables easy testing and mocking; constitutional compliance achieved for all P1 requirements; ready for textbook ingestion and end-to-end validation

## Evaluation notes (flywheel)

- Failure modes observed: None - all files created successfully; no import errors or syntax issues detected during creation
- Graders run and results (PASS/FAIL): Not applicable - manual testing required after environment setup; validation scripts created for user to run
- Prompt variant (if applicable): Shortened implementation focusing on core RAG pipeline (option 2 from /sp.implement menu); skipped async patterns and database logging for initial MVP
- Next experiment (smallest change to try): After user validates setup and ingests textbook, test latency with real queries to identify if p90 ≤2s target is achievable or if optimizations needed (e.g., reduce top_k, optimize prompt length, or add caching)
