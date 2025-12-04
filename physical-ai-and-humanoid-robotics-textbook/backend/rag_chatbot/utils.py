import os
import re
from typing import List
from fastembed import TextEmbedding

# No longer need to load dotenv for GOOGLE_API_KEY
# from dotenv import load_dotenv

# --- FastEmbed Initialization ---
# You can specify a different model from the FastEmbed documentation
embedding_model = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")
EMBEDDING_DIMENSION = 384  # Dimension for BAAI/bge-small-en-v1.5

def get_embeddings(texts: List[str]) -> List[List[float]]:
    """
    Generates embeddings for a list of texts using FastEmbed.
    """
    try:
        # FastEmbed's embed method returns a list of numpy arrays.
        # We convert them to a list of lists of floats.
        embeddings = embedding_model.embed(texts)
        return [embedding.tolist() for embedding in embeddings]
    except Exception as e:
        print(f"Error generating embeddings with FastEmbed: {e}")
        return [[] for _ in texts] # Return empty embeddings on error


def chunk_text(text: str, chunk_size: int = 500, chunk_overlap: int = 50) -> List[str]:
    """
    Splits a given text into smaller chunks without breaking sentences,
    aiming for chunks of max_chunk_size characters.
    """
    # A very basic text chunker. For production, consider more advanced methods
    # from libraries like LangChain or LlamaIndex.
    chunks = []
    if len(text) <= chunk_size:
        return [text]
    
    words = text.split()
    current_chunk = []
    current_length = 0

    for word in words:
        if current_length + len(word) + 1 > chunk_size and current_chunk:
            chunks.append(" ".join(current_chunk))
            # Start new chunk with some overlap
            overlap_words = current_chunk[-int(chunk_overlap / (len(current_chunk[-1]) + 1)):] if current_chunk else []
            current_chunk = overlap_words + [word]
            current_length = sum(len(w) for w in current_chunk) + len(current_chunk) - 1
        else:
            current_chunk.append(word)
            current_length += len(word) + 1
    
    if current_chunk:
        chunks.append(" ".join(current_chunk))
    return chunks
