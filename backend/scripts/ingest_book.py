"""Book ingestion script.

Chunks a textbook PDF, generates embeddings, and stores in Qdrant.
"""
import argparse
import sys
from pathlib import Path
from uuid import UUID, uuid4
from typing import List, Dict, Any
import tiktoken

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.services.embeddings import embedding_service
from src.services.vector_store import vector_store_service
from src.utils.config import config


class BookChunker:
    """Handles text chunking with overlap."""

    def __init__(self, chunk_size: int = 750, overlap: int = 75):
        """Initialize chunker.

        Args:
            chunk_size: Target tokens per chunk
            overlap: Overlapping tokens between chunks
        """
        self.chunk_size = chunk_size
        self.overlap = overlap
        self.encoding = tiktoken.get_encoding("cl100k_base")

    def chunk_text(
        self,
        text: str,
        book_id: UUID,
        chapter: int,
        section: str = None,
        page: int = None
    ) -> List[Dict[str, Any]]:
        """Chunk text into overlapping segments.

        Args:
            text: Input text to chunk
            book_id: Book identifier
            chapter: Chapter number
            section: Section title (optional)
            page: Starting page number (optional)

        Returns:
            List of chunk dicts with metadata
        """
        # Tokenize the text
        tokens = self.encoding.encode(text)
        chunks = []

        # Split into chunks with overlap
        start = 0
        chunk_index = 0

        while start < len(tokens):
            end = start + self.chunk_size
            chunk_tokens = tokens[start:end]

            # Decode back to text
            chunk_text = self.encoding.decode(chunk_tokens)

            chunk = {
                "chunk_id": uuid4(),
                "book_id": book_id,
                "chapter": chapter,
                "section": section,
                "page": page,
                "chunk_index": chunk_index,
                "text": chunk_text,
                "token_count": len(chunk_tokens),
            }

            chunks.append(chunk)

            # Move to next chunk with overlap
            start += self.chunk_size - self.overlap
            chunk_index += 1

        return chunks


def ingest_book(
    text: str,
    book_id: UUID,
    title: str,
    author: str,
    chapter: int = 1,
    section: str = None
) -> None:
    """Ingest a book into the RAG system.

    Args:
        text: Full text content
        book_id: Unique book identifier
        title: Book title
        author: Author name
        chapter: Chapter number
        section: Section title (optional)
    """
    print(f"Starting ingestion for: {title} by {author}")
    print(f"Book ID: {book_id}")

    # Ensure Qdrant collection exists
    print("Ensuring Qdrant collection exists...")
    vector_store_service.ensure_collection()

    # Chunk the text
    print(f"Chunking text (chunk_size={config.CHUNK_SIZE}, overlap={config.CHUNK_OVERLAP})...")
    chunker = BookChunker(
        chunk_size=config.CHUNK_SIZE,
        overlap=config.CHUNK_OVERLAP
    )

    chunks = chunker.chunk_text(
        text=text,
        book_id=book_id,
        chapter=chapter,
        section=section
    )

    print(f"Created {len(chunks)} chunks")

    # Generate embeddings in batches
    batch_size = 96  # Cohere limit
    print(f"Generating embeddings (batch_size={batch_size})...")

    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i + batch_size]
        texts = [chunk["text"] for chunk in batch]

        # Get embeddings (use sync version for scripts)
        embeddings = embedding_service.embed_chunks_sync(texts)

        # Add embeddings to chunks
        for chunk, embedding in zip(batch, embeddings):
            chunk["embedding"] = embedding

        print(f"  Processed {min(i + batch_size, len(chunks))}/{len(chunks)} chunks")

    # Upload to Qdrant
    print("Uploading to Qdrant...")
    vector_store_service.upsert_chunks(chunks)

    print("✓ Ingestion complete!")
    print(f"  Total chunks: {len(chunks)}")
    print(f"  Book ID: {book_id}")


def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(description="Ingest a textbook into the RAG system")
    parser.add_argument("--text-file", type=str, required=True, help="Path to text file")
    parser.add_argument("--book-id", type=str, help="Book UUID (generated if not provided)")
    parser.add_argument("--title", type=str, required=True, help="Book title")
    parser.add_argument("--author", type=str, required=True, help="Author name")
    parser.add_argument("--chapter", type=int, default=1, help="Chapter number")
    parser.add_argument("--section", type=str, help="Section title")

    args = parser.parse_args()

    # Load text file
    text_path = Path(args.text_file)
    if not text_path.exists():
        print(f"Error: File not found: {text_path}")
        sys.exit(1)

    with open(text_path, "r", encoding="utf-8") as f:
        text = f.read()

    # Generate or parse book ID
    if args.book_id:
        try:
            book_id = UUID(args.book_id)
        except ValueError:
            print(f"Error: Invalid UUID: {args.book_id}")
            sys.exit(1)
    else:
        book_id = uuid4()

    # Ingest the book
    try:
        ingest_book(
            text=text,
            book_id=book_id,
            title=args.title,
            author=args.author,
            chapter=args.chapter,
            section=args.section
        )
    except Exception as e:
        print(f"Error during ingestion: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
