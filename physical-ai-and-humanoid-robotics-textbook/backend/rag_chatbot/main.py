import os
import asyncio
import asyncpg  # pyright: ignore[reportMissingImports]
from typing import List, Dict, Any
from fastapi import FastAPI, HTTPException  # pyright: ignore[reportMissingImports]
from fastapi.middleware.cors import CORSMiddleware  # pyright: ignore[reportMissingImports]
from pydantic import BaseModel  # pyright: ignore[reportMissingImports]
from qdrant_client import QdrantClient, models  # pyright: ignore[reportMissingImports]
from qdrant_client.http.exceptions import UnexpectedResponse  # pyright: ignore[reportMissingImports]
import google.generativeai as genai  # pyright: ignore[reportMissingImports]
from dotenv import load_dotenv  # pyright: ignore[reportMissingImports]

from .utils import get_embeddings, chunk_text, EMBEDDING_DIMENSION

load_dotenv()  # Load environment variables

# Explicitly check for API key in .env file first
if os.path.exists('.env'):
    # Load again to ensure .env values take precedence over system variables
    from dotenv import dotenv_values
    env_vars = dotenv_values('.env')
    os.environ['GOOGLE_API_KEY'] = env_vars.get('GOOGLE_API_KEY', os.environ.get('GOOGLE_API_KEY', ''))

app = FastAPI()

# ------------------ CORS ------------------
origins = [
    os.getenv("FRONTEND_URL", "http://localhost:3000"),
    "http://localhost:8000",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ------------------ Google Gemini ------------------
GOOGLE_API_KEY = os.getenv("GOOGLE_API_KEY")
if not GOOGLE_API_KEY:
    raise ValueError("GOOGLE_API_KEY not set")
genai.configure(api_key=GOOGLE_API_KEY)

# ------------------ Database ------------------
DATABASE_URL = os.getenv("DATABASE_URL")
_pg_connection: asyncpg.Connection | None = None

async def get_db_connection():
    global _pg_connection
    if _pg_connection is None or _pg_connection.is_closed():
        _pg_connection = await asyncpg.connect(DATABASE_URL)
    return _pg_connection

# ------------------ Qdrant Singleton ------------------
QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
_qdrant_client: QdrantClient | None = None

def get_qdrant_client() -> QdrantClient:
    global _qdrant_client
    if _qdrant_client is None:
        _qdrant_client = QdrantClient(
            url=QDRANT_HOST,
            api_key=QDRANT_API_KEY,
            prefer_grpc=True,
        )
    return _qdrant_client

# ------------------ Startup / Shutdown ------------------
@app.on_event("startup")
async def startup_event():
    # Postgres
    try:
        conn = await get_db_connection()
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS book_content (
                id SERIAL PRIMARY KEY,
                chapter VARCHAR(255),
                section TEXT,
                content TEXT,
                url TEXT UNIQUE
            );
        """)
        print("Postgres table ensured.")
    except Exception as e:
        print(f"Postgres startup error: {e}")

    # Qdrant
    try:
        client = get_qdrant_client()
        client.get_collections()
        print("Connected to Qdrant Cloud.")
    except Exception as e:
        print(f"Qdrant startup error: {e}")

@app.on_event("shutdown")
async def shutdown_event():
    global _pg_connection, _qdrant_client
    if _pg_connection and not _pg_connection.is_closed():
        await _pg_connection.close()
        print("Closed Postgres connection.")
    if _qdrant_client:
        _qdrant_client.close()
        _qdrant_client = None
        print("Closed Qdrant client.")

# ------------------ Test Routes ------------------
@app.get("/")
async def root():
    return {"message": "RAG Chatbot Backend running!"}

@app.get("/test-db-connection")
async def test_db_connection():
    try:
        conn = await get_db_connection()
        result = await conn.fetchval("SELECT 1;")
        return {"status": "success", "db_result": result}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@app.get("/test-qdrant-connection")
async def test_qdrant_connection():
    try:
        client = get_qdrant_client()
        collections = client.get_collections()
        return {"status": "success", "collections": collections.collections}
    except Exception as e:
        return {"status": "error", "message": str(e)}

# ------------------ Collection Management ------------------
class CreateCollectionRequest(BaseModel):
    collection_name: str
    vector_size: int = EMBEDDING_DIMENSION

@app.post("/create-collection")
async def create_qdrant_collection(request: CreateCollectionRequest):
    try:
        client = get_qdrant_client()
        client.recreate_collection(
            collection_name=request.collection_name,
            vectors_config=models.VectorParams(size=request.vector_size, distance=models.Distance.COSINE)
        )
        return {"status": "success", "message": f"Collection '{request.collection_name}' created/recreated."}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# ------------------ Upsert Content ------------------
class UpsertContentRequest(BaseModel):
    collection_name: str
    documents: List[Dict[str, Any]]

@app.post("/upsert-content")
async def upsert_content_to_qdrant(request: UpsertContentRequest):
    try:
        client = get_qdrant_client()
        texts = [doc["text"] for doc in request.documents]
        metadatas = [doc.get("metadata", {}) for doc in request.documents]
        embeddings = get_embeddings(texts)

        points = []
        for i, (text, metadata, emb) in enumerate(zip(texts, metadatas, embeddings)):
            point_id = metadata.get("id", i)
            points.append(models.PointStruct(
                id=point_id,
                vector=emb,
                payload={"text": text, **metadata}
            ))

        op_info = client.upsert(collection_name=request.collection_name, points=points, wait=True)
        return {"status": "success", "upserted": len(points), "operation_id": op_info.operation_id}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# ------------------ Helper: Retrieve Context ------------------
def retrieve_context(query_text: str, collection_name: str, limit: int = 5) -> list[dict]:
    client = get_qdrant_client()
    query_emb = get_embeddings([query_text])[0]

    try:
        search_result = client.search_points(
            collection_name=collection_name,
            query_vector=query_emb,
            limit=limit,
            with_payload=True
        )
    except Exception as e:
        print(f"Qdrant search error: {e}")
        return []

    context = []
    for hit in search_result:
        context.append({
            "text": hit.payload.get("text", ""),
            "score": hit.score,
            "metadata": {k: v for k, v in hit.payload.items() if k != "text"}
        })
    return context

# ------------------ Chat Endpoint ------------------
class ChatRequest(BaseModel):
    query: str
    collection_name: str = "book_content"

@app.post("/chat")
async def chat(request: ChatRequest):
    try:
        context = retrieve_context(request.query, request.collection_name)
        context_str = "\n\n".join([doc["text"] for doc in context])

        # Build prompt as a single string
        prompt_text = (
            "You are an AI assistant. Use the context below to answer the user's question.\n\n"
            f"Context:\n{context_str}\n\n"
            f"User Question: {request.query}"
        )

        model = genai.GenerativeModel("gemini-2.5-flash")
        answer = await safe_generate_content(model, prompt_text)

        return {"response": answer, "context": context}
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Chat failed: {e}")

# ------------------ Query Selected Text ------------------
class QuerySelectedTextRequest(BaseModel):
    selected_text: str
    user_question: str | None = None
    collection_name: str = "book_content"

@app.post("/query_selected_text")
async def query_selected_text(request: QuerySelectedTextRequest):
    try:
        query = request.user_question if request.user_question else request.selected_text
        context = retrieve_context(query, request.collection_name)
        context_str = "\n\n".join([doc["text"] for doc in context])

        prompt_text = (
            "You are an AI assistant. Answer based on the selected text and relevant context.\n\n"
            f"Selected Text:\n{request.selected_text}\n\n"
            f"Context:\n{context_str}\n\n"
            f"User Question: {request.user_question or 'Explain the selected text.'}"
        )

        model = genai.GenerativeModel("gemini-2.5-flash")
        answer = await safe_generate_content(model, prompt_text)

        return {"response": answer, "context": context}
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Query failed: {e}")

# ------------------ Generate Response Helper ------------------
async def generate_response(prompt_messages: List[Dict[str, str]]) -> str:
    model = genai.GenerativeModel("gemini-2.5-flash")
    # Concatenate all messages into a single prompt string
    prompt_text = "\n\n".join([msg["content"] for msg in prompt_messages])
    resp = await model.generate_content_async(prompt_text)
    return resp.text if resp else "No response generated."


# Error handling wrapper for Google API calls
async def safe_generate_content(model: genai.GenerativeModel, prompt: str) -> str:
    try:
        resp = await model.generate_content_async(prompt)
        return resp.text if resp else "No response generated."
    except Exception as e:
        error_msg = str(e)
        if "403" in error_msg or "Permission denied" in error_msg or "CONSUMER_SUSPENDED" in error_msg:
            raise HTTPException(
                status_code=500,
                detail="Google API key issue. Please check that your API key is valid and not suspended."
            )
        else:
            raise e
