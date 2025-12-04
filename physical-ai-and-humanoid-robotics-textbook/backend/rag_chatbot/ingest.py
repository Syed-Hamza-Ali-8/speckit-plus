import os
import asyncio
import glob
import re
from typing import List
import asyncpg  # pyright: ignore[reportMissingImports]
from qdrant_client import QdrantClient, models  # pyright: ignore[reportMissingImports]
from dotenv import load_dotenv  # pyright: ignore[reportMissingImports]

from .utils import get_embeddings, chunk_text, EMBEDDING_DIMENSION

# Load environment variables
load_dotenv(os.path.join(os.path.dirname(__file__), ".env"))

# --- Config ---
DATABASE_URL = os.getenv("DATABASE_URL")
QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_content")

if not all([DATABASE_URL, QDRANT_HOST, QDRANT_API_KEY]):
    raise ValueError("Missing one or more required environment variables")

# ------------------ Qdrant Singleton ------------------
_qdrant_client: QdrantClient | None = None

def get_qdrant_client() -> QdrantClient:
    global _qdrant_client
    if _qdrant_client is None:
        _qdrant_client = QdrantClient(
            url=QDRANT_HOST,
            api_key=QDRANT_API_KEY,
            prefer_grpc=True
        )
    return _qdrant_client

# ---------------------------------------
# Database Connection
# ---------------------------------------
async def get_db_connection():
    return await asyncpg.connect(DATABASE_URL)

async def setup_db(conn):
    await conn.execute("""
        CREATE TABLE IF NOT EXISTS book_content (
            id SERIAL PRIMARY KEY,
            file_path TEXT UNIQUE,
            title VARCHAR(255),
            chapter VARCHAR(255),
            section TEXT,
            url TEXT,
            last_modified TIMESTAMP WITH TIME ZONE
        );
    """)

# ---------------------------------------
# Document Ingestion Logic
# ---------------------------------------
async def ingest_document(file_path: str, conn: asyncpg.Connection):
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Title
    title_match = re.search(r'^#\s*(.*)', content, re.MULTILINE)
    title = title_match.group(1).strip() if title_match else os.path.basename(file_path)

    # Clean content
    clean_content = re.sub(r'---\s*[\s\S]*?---\s*', '', content, 1)
    clean_content = re.sub(r'#+\s*', '', clean_content)
    clean_content = re.sub(r'[^\w\s\.\,\?\!]', '', clean_content)
    clean_content = re.sub(r'\s+', ' ', clean_content).strip()

    # Determine URL
    doc_path_parts = file_path.replace("\\", "/").split('/')
    if "docs" in doc_path_parts:
        url_segment = "/docs/" + "/".join(doc_path_parts[doc_path_parts.index("docs") + 1:]).replace(".md", "")
    elif "blog" in doc_path_parts:
        url_segment = "/blog/" + "/".join(doc_path_parts[doc_path_parts.index("blog") + 1:]).replace(".md", "")
    else:
        url_segment = "/" + os.path.basename(file_path).replace(".md", "")

    # Chapter & Section
    chapter = doc_path_parts[doc_path_parts.index("docs") + 1] if "docs" in doc_path_parts else "Unknown"
    section = title

    # Chunk + Embeddings
    chunks = chunk_text(clean_content)
    embeddings = get_embeddings(chunks)

    points_to_upsert = []
    for idx, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
        points_to_upsert.append(
            models.PointStruct(
                id=hash(f"{file_path}-{idx}") & 0x7FFFFFFF,  # preserves existing IDs
                vector=embedding,
                payload={
                    "text": chunk,
                    "file_path": file_path,
                    "title": title,
                    "chapter": chapter,
                    "section": section,
                    "url": url_segment,
                    "chunk_index": idx,
                    "total_chunks": len(chunks),
                },
            )
        )

    # Upload to Qdrant
    try:
        client = get_qdrant_client()
        client.upsert(
            collection_name=COLLECTION_NAME,
            points=points_to_upsert,
            wait=True
        )
        print(f"Upserted {len(points_to_upsert)} points → Qdrant ({file_path})")
    except Exception as e:
        print(f"[ERROR] Qdrant upsert failed for {file_path}: {e}")

    # Store metadata in Postgres
    try:
        await conn.execute("""
            INSERT INTO book_content (file_path, title, chapter, section, url, last_modified)
            VALUES ($1, $2, $3, $4, $5, NOW())
            ON CONFLICT (file_path) DO UPDATE
            SET title = $2, chapter = $3, section = $4, url = $5, last_modified = NOW();
        """, file_path, title, chapter, section, url_segment)
        print(f"Updated Postgres for {file_path}")
    except Exception as e:
        print(f"[ERROR] Postgres update failed for {file_path}: {e}")

# ---------------------------------------
# Main Ingestion Flow
# ---------------------------------------
async def main():
    print(f"🚀 Starting ingestion for collection: {COLLECTION_NAME}")

    # Postgres connect
    pg_conn = await get_db_connection()
    await setup_db(pg_conn)

    # Create Qdrant collection if it doesn't exist
    client = get_qdrant_client()
    if not client.collection_exists(collection_name=COLLECTION_NAME):
        try:
            client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=models.VectorParams(
                    size=EMBEDDING_DIMENSION,
                    distance=models.Distance.COSINE
                ),
            )
            print(f"Qdrant collection '{COLLECTION_NAME}' created with {EMBEDDING_DIMENSION} dims.")
        except Exception as e:
            print(f"[ERROR] Failed to create Qdrant collection: {e}")
            return
    else:
        print(f"Qdrant collection '{COLLECTION_NAME}' already exists.")

    # -------------------------------
    # Find markdown files
    # -------------------------------
    ROOT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../"))
    docs_pattern = os.path.join(ROOT_PATH, "book/docs/module-*/**/*.md")
    blog_pattern = os.path.join(ROOT_PATH, "book/blog/**/*.md")

    markdown_files = glob.glob(docs_pattern, recursive=True)
    markdown_files.extend(glob.glob(blog_pattern, recursive=True))
    markdown_files = [f for f in markdown_files if not os.path.basename(f).startswith("_category_")]

    if not markdown_files:
        print(f"⚠️ No markdown files found under {docs_pattern} or {blog_pattern}.")
        return

    print(f"✅ Found {len(markdown_files)} markdown files. Starting ingestion...")
    for md_file in markdown_files:
        print(f"📄 Processing: {md_file}")
        await ingest_document(md_file, pg_conn)

    await pg_conn.close()
    print("🎉 Ingestion completed successfully.")

if __name__ == "__main__":
    asyncio.run(main())
