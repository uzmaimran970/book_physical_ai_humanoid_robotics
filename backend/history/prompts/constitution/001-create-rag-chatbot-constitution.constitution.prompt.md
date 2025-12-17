---
id: 001
title: Create RAG Chatbot Constitution
stage: constitution
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-textbook-generation
user: Uzma Imran
command: /sp.constitution
labels: ["constitution", "rag-chatbot", "initialization", "principles", "governance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A (Constitution documentation)
---

## Prompt

```text
/sp.constitution

Project: Integrated RAG Chatbot for Published Book

Core principles:
- Accuracy: Answer only from text selected by the user
- Context-awareness: Must reference relevant sections/pages
- Reliability: Avoid hallucinations; answer must be factual
- User experience: Responses should be clear, concise, and informative
- Traceability: Each answer must be traceable to retrieved text

Key standards:
- Use Cohere API for embeddings (text → vectors)
- Use Qdrant for vector storage and search
- Claude CLI generates the final response based on retrieved context
- FastAPI exposes the chatbot API endpoints
- Neon Serverless Postgres stores user queries, retrieved context, and chatbot answers
- Chatbot must handle multi-user queries asynchronously
- Responses must always respect the book content; no external info

Constraints:
- Only use Cohere embeddings, no OpenAI API
- Responses limited to the retrieved content
- Latency per query ≤ 2 seconds for demo purposes
- Maintain privacy of user data
- Handle books of arbitrary length (chunking required)

Success criteria:
- Every answer is based on retrieved text
- System logs all queries and responses in Neon DB
- Vector search retrieves correct context chunks
- Claude CLI outputs coherent and professional responses
- Chatbot ready for hackathon demo
```

## Response snapshot

Created comprehensive constitution for Integrated RAG Chatbot project with 7 core principles and complete governance structure.

**Version**: 1.0.0 (Initial release)
**File**: `.specify/memory/constitution.md`

### Core Principles Established:

1. **Accuracy & Factual Fidelity (NON-NEGOTIABLE)**: Answers ONLY from retrieved book content; no hallucinations
2. **Context-Awareness & Traceability**: All answers must reference source sections/pages with citations
3. **Reliability & Query Logging**: All queries/responses logged in Neon Postgres; multi-user concurrent support
4. **Performance & Latency Constraints**: ≤ 2 seconds per query for demo
5. **Privacy & Data Handling**: User data confidential; no cross-user leakage; environment variables for secrets
6. **Technology Stack Adherence**: Cohere + Qdrant + Claude + FastAPI + Neon (no OpenAI)
7. **Chunking & Scalability**: Books of arbitrary length with 500-1000 token chunks, 50-100 token overlap

### Sections Added:

- **Technical Stack & Constraints**: Approved stack, constraints, NFRs
- **Development Workflow**: Test-first discipline, code quality standards, version control, simplicity principles
- **Governance**: Amendment process, compliance requirements, versioning policy

### Template Consistency:

- ✅ plan-template.md reviewed - Constitution Check section aligns
- ✅ spec-template.md reviewed - Requirements section aligns
- ✅ tasks-template.md reviewed - Task categorization aligns

## Outcome

- ✅ Impact: Established foundational governance document for RAG chatbot backend; all 7 principles operationalized with clear requirements and rationale
- 🧪 Tests: N/A (documentation artifact)
- 📁 Files: Created `.specify/memory/constitution.md` (201 lines)
- 🔁 Next prompts: `/sp.specify` to create first feature specification, `/sp.plan` to design implementation architecture
- 🧠 Reflection: Constitution provides clear constraints (no OpenAI, latency <2s, Cohere+Qdrant+Claude stack) and non-negotiable principles (accuracy, traceability, privacy). Ready for feature development.

## Evaluation notes (flywheel)

- Failure modes observed: None - first constitution creation for this project
- Graders run and results (PASS/FAIL): Manual validation - all placeholders filled, no bracketed tokens remaining, version/dates coherent (PASS)
- Prompt variant (if applicable): Standard /sp.constitution workflow
- Next experiment (smallest change to try): Validate principles during first `/sp.plan` execution to ensure Constitution Check gates are effective
