from dotenv import load_dotenv
import os
from typing import TypedDict, List
from datetime import datetime
import requests
from bs4 import BeautifulSoup
from urllib.parse import urlparse, urljoin
import cohere # Added import
from qdrant_client import QdrantClient, models # Added imports
import logging # Added import

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# --- Constants for Chunking ---
CHUNK_SIZE = 512  # tokens (approximate, character count is used here)
CHUNK_OVERLAP = 50  # characters

class VectorRecord(TypedDict):
    text: str
    url: str
    section_title: str
    chunk_index: int
    processed_at: str # ISO 8601 format

def load_config() -> dict:
    """Loads environment variables from a .env file and returns them as a dictionary."""
    load_dotenv()
    config = {
        "TARGET_SITE_URL": os.getenv("TARGET_SITE_URL"),
        "COHERE_API_KEY": os.getenv("COHERE_API_KEY"),
        "QDRANT_URL": os.getenv("QDRANT_URL"),
        "QDRANT_API_KEY": os.getenv("QDRANT_API_KEY"),
        "QDRANT_COLLECTION_NAME": os.getenv("QDRANT_COLLECTION_NAME", "physical-ai-book-v1"),
    }
    # Basic validation
    for key, value in config.items():
        if key not in ["QDRANT_COLLECTION_NAME"] and value is None:
            raise ValueError(f"Missing environment variable: {key}")
    return config

def get_sitemap_urls(site_url: str) -> List[str]:
    """
    Fetches the sitemap.xml from the given site_url and extracts all URLs.

    Args:
        site_url (str): The base URL of the website.

    Returns:
        List[str]: A list of URLs found in the sitemap.
    """
    sitemap_url = f"{site_url.rstrip('/')}/sitemap.xml"
    response = requests.get(sitemap_url)
    response.raise_for_status()
    soup = BeautifulSoup(response.content, 'xml')
    urls = [loc.text for loc in soup.find_all('loc')]
    return urls

def fetch_and_extract_content(url: str) -> str:
    """
    Fetches the content of a given URL and extracts the main text from the <main> HTML tag.

    Args:
        url (str): The URL of the page to fetch and extract content from.

    Returns:
        str: The extracted main text content, or an empty string if not found.
    """
    response = requests.get(url)
    response.raise_for_status()
    soup = BeautifulSoup(response.content, 'html.parser')

    # Extract title for section_title fallback
    title_tag = soup.find('title')
    page_title = title_tag.get_text(strip=True) if title_tag else "No Title"

    main_content = soup.find('main')
    if main_content:
        # Remove unwanted elements like nav, footer, header, script, style, aside, etc.
        for unwanted in main_content(['nav', 'footer', 'header', 'script', 'style', 'aside', '.docitem-container__pagination']):
            unwanted.decompose()
        return main_content.get_text(separator='\n', strip=True)
    return "" # Return empty string if main content not found or extracted text is empty

def chunk_text(text: str) -> List[str]:
    """
    Splits the given text into smaller, overlapping chunks using a recursive character splitting strategy.

    Args:
        text (str): The input text to be chunked.

    Returns:
        List[str]: A list of text chunks.
    """
    chunks = []
    # Attempt to split by paragraphs first
    paragraphs = text.split('\n\n')
    for para in paragraphs:
        if len(para) > CHUNK_SIZE:
            # If paragraph is too long, split by sentences
            sentences = para.split('.')
            current_chunk = ""
            for sentence in sentences:
                if len(current_chunk) + len(sentence) + 1 > CHUNK_SIZE:
                    if current_chunk:
                        chunks.append(current_chunk.strip() + '.')
                    current_chunk = sentence
                else:
                    current_chunk += sentence + '.'
            if current_chunk:
                chunks.append(current_chunk.strip())
        else:
            chunks.append(para.strip())
    
    final_chunks = []
    # Apply overlap
    for i in range(len(chunks)):
        if i > 0:
            overlap_start = max(0, len(chunks[i-1]) - CHUNK_OVERLAP)
            # Ensure overlap doesn't create tiny chunks at the start
            overlap_text = chunks[i-1][overlap_start:]
            final_chunks.append(overlap_text + chunks[i])
        else:
            final_chunks.append(chunks[i])

    return [chunk for chunk in final_chunks if chunk] # Filter out empty chunks

def get_cohere_embeddings(chunks: List[str], cohere_api_key: str) -> List[List[float]]:
    """
    Generates Cohere embeddings for a list of text chunks.

    Args:
        chunks (List[str]): A list of text chunks to embed.
        cohere_api_key (str): The API key for Cohere.

    Returns:
        List[List[float]]: A list of embeddings, where each embedding is a list of floats.
    """
    try:
        co = cohere.Client(cohere_api_key)
        response = co.embed(
            texts=chunks,
            model="embed-english-v3.0",
            input_type="search_document"
        )
        return response.embeddings
    except cohere.CohereError as e:
        logging.error(f"Error generating Cohere embeddings: {e}")
        return []
    except Exception as e:
        logging.error(f"An unexpected error occurred during Cohere embedding: {e}")
        return []

def upsert_to_qdrant(records: List[VectorRecord], config: dict, embeddings: List[List[float]]):
    """
    Upserts a list of vector records and their embeddings to Qdrant.

    Args:
        records (List[VectorRecord]): A list of VectorRecord objects to upsert.
        config (dict): Configuration dictionary containing Qdrant URL, API key, and collection name.
        embeddings (List[List[float]]): A list of corresponding embeddings for the records.
    """
    try:
        client = QdrantClient(
            url=config["QDRANT_URL"],
            api_key=config["QDRANT_API_KEY"],
        )
        
        points = []
        for i, record in enumerate(records):
            points.append(
                models.PointStruct(
                    id=i, # Simple sequential ID, adjust for more robust ID generation
                    vector=embeddings[i],
                    payload=record,
                )
            )
        
        client.upsert(
            collection_name=config["QDRANT_COLLECTION_NAME"],
            wait=True,
            points=points,
        )
    except Exception as e:
        logging.error(f"Error upserting to Qdrant: {e}")

def run_pipeline() -> None:
    """
    Orchestrates the entire RAG content ingestion pipeline.
    Fetches sitemap URLs, extracts content, chunks text, generates embeddings,
    and upserts records to Qdrant.
    """
    config = load_config()
    site_url = config["TARGET_SITE_URL"]
    cohere_api_key = config["COHERE_API_KEY"]

    logging.info(f"Starting RAG content ingestion pipeline for {site_url}...")

    # Initialize Qdrant client for collection management
    qdrant_client_init = QdrantClient(
        url=config["QDRANT_URL"],
        api_key=config["QDRANT_API_KEY"],
    )
    # Ensure collection exists and is fresh for updates
    qdrant_client_init.recreate_collection(
        collection_name=config["QDRANT_COLLECTION_NAME"],
        vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE), # Cohere v3 embedding size is 1024
    )
    logging.info(f"Qdrant collection '{config['QDRANT_COLLECTION_NAME']}' recreated.")

    urls = get_sitemap_urls(site_url)
    logging.info(f"Found {len(urls)} URLs in sitemap.")

    all_records: List[VectorRecord] = []
    all_chunks: List[str] = []
    current_chunk_index = 0

    for url in urls:
        try:
            logging.info(f"Processing URL: {url}")
            content = fetch_and_extract_content(url)
            if not content:
                logging.warning(f"  No main content found for {url}, skipping.")
                continue

            chunks = chunk_text(content)
            logging.info(f"  Split into {len(chunks)} chunks.")

            for chunk in chunks:
                # Assuming section_title can be extracted or is a placeholder for now
                # A more sophisticated parser would get actual section titles
                section_title = "Document Content" # Placeholder
                
                all_chunks.append(chunk)
                all_records.append(
                    VectorRecord(
                        text=chunk,
                        url=url,
                        section_title=section_title,
                        chunk_index=current_chunk_index,
                        processed_at=datetime.now().isoformat()
                    )
                )
                current_chunk_index += 1
        except requests.exceptions.RequestException as e:
            logging.error(f"Error fetching or processing URL {url}: {e}")
        except Exception as e:
            logging.error(f"An unexpected error occurred for URL {url}: {e}")
            
    logging.info(f"Total chunks to embed: {len(all_chunks)}")
    if not all_chunks:
        logging.warning("No chunks to process. Exiting.")
        return

    embeddings = get_cohere_embeddings(all_chunks, cohere_api_key)
    logging.info(f"Generated {len(embeddings)} embeddings.")

    upsert_to_qdrant(all_records, config, embeddings)
    logging.info(f"Successfully upserted {len(all_records)} records to Qdrant collection '{config['QDRANT_COLLECTION_NAME']}'.")

    logging.info("Pipeline finished.")

if __name__ == "__main__":
    run_pipeline()
