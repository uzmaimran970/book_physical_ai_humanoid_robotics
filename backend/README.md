# RAG Chatbot for Physical AI Textbook

A Retrieval-Augmented Generation (RAG) chatbot that answers questions from a Physical AI & Humanoid Robotics textbook with accurate citations.

## Features

- **Accurate Responses**: Answers based strictly on textbook content
- **Citation Support**: Includes chapter, section, and page references
- **Fast Performance**: <2 second response time for most queries
- **Concurrent Access**: Handles multiple users simultaneously
- **Query Analytics**: Logs all interactions for insights

## Tech Stack

- **Framework**: FastAPI (Python 3.11+)
- **Embeddings**: Cohere embed-english-v3.0 (1024 dimensions)
- **Vector Store**: Qdrant Cloud (cosine similarity)
- **LLM**: Anthropic Claude Sonnet 4
- **Database**: Neon Serverless Postgres
- **Chunking**: tiktoken (750 tokens, 75 overlap)

## Quick Start

### 1. Prerequisites

- Python 3.11 or higher
- API keys for Cohere, Qdrant, Anthropic, and Neon DB

### 2. Installation

```bash
# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Configuration

```bash
# Copy environment template
cp .env.example .env

# Edit .env with your API keys
nano .env
```

### 4. Run the API

```bash
# Start the server
python -m src.main

# Or with uvicorn directly
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at `http://localhost:8000`

Interactive docs: `http://localhost:8000/docs`

## API Endpoints

### POST /api/v1/query

Query the textbook:

```bash
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are IMU sensors?",
    "top_k": 3
  }'
```

Response:

```json
{
  "query_id": "123e4567-e89b-12d3-a456-426614174000",
  "answer": "IMU sensors are inertial measurement units...",
  "citations": [
    {
      "chapter": 3,
      "section": "Sensors",
      "page": 45,
      "chunk_id": "...",
      "relevance_score": 0.89
    }
  ],
  "latency_ms": 1250,
  "status": "success"
}
```

### GET /api/v1/health

Check service health:

```bash
curl http://localhost:8000/api/v1/health
```

## Project Structure

```
backend/
├── src/
│   ├── api/
│   │   └── routes.py          # API endpoints
│   ├── models/
│   │   └── entities.py        # Pydantic models
│   ├── services/
│   │   ├── embeddings.py      # Cohere wrapper
│   │   ├── vector_store.py    # Qdrant wrapper
│   │   ├── llm.py             # Claude wrapper
│   │   └── rag_pipeline.py    # RAG orchestration
│   ├── utils/
│   │   ├── config.py          # Configuration
│   │   └── error_handling.py  # Custom exceptions
│   └── main.py                # FastAPI app
├── scripts/
│   └── ingest_book.py         # Book ingestion script
├── requirements.txt
├── .env.example
└── README.md
```

## Ingesting a Textbook

To add a textbook to the system:

```bash
python scripts/ingest_book.py --pdf path/to/textbook.pdf --book-id <uuid>
```

This will:
1. Extract text from PDF
2. Chunk text (750 tokens, 75 overlap)
3. Generate embeddings with Cohere
4. Store vectors in Qdrant

## Development

### Running Tests

```bash
pytest tests/
```

### Code Quality

```bash
# Format code
black src/

# Lint code
pylint src/
```

## Configuration Options

Environment variables in `.env`:

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `QDRANT_URL` | Yes | - | Qdrant cluster URL |
| `QDRANT_API_KEY` | Yes | - | Qdrant API key |
| `COHERE_API_KEY` | Yes | - | Cohere API key |
| `ANTHROPIC_API_KEY` | Yes | - | Anthropic API key |
| `NEON_DB_URL` | Yes | - | Neon database URL |
| `CHUNK_SIZE` | No | 750 | Tokens per chunk |
| `CHUNK_OVERLAP` | No | 75 | Token overlap |
| `TOP_K_DEFAULT` | No | 3 | Default results |
| `LOG_LEVEL` | No | INFO | Logging level |

## Performance

- **Latency**: p90 <2 seconds
- **Concurrent Users**: 10+ simultaneous queries
- **Accuracy**: Responses grounded in textbook content

## Troubleshooting

### API Key Errors

Ensure all API keys are set in `.env`:

```bash
# Check environment
python -c "from src.utils.config import config; config.validate()"
```

### Qdrant Connection Issues

Verify Qdrant cluster is accessible:

```bash
curl -H "api-key: YOUR_KEY" https://your-cluster.qdrant.io/collections
```

### Slow Response Times

Check:
- Network latency to Qdrant/Cohere/Anthropic
- Qdrant collection size
- Token count in chunks

## License

MIT License

## Support

For issues or questions, please open an issue on GitHub.
