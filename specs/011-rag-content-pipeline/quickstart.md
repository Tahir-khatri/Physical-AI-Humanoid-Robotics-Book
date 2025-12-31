# Quickstart: RAG Content Pipeline

**Date**: 2025-12-30
**Feature**: `011-rag-content-pipeline`

This guide provides instructions on how to set up and run the content ingestion pipeline.

## 1. Prerequisites

- Python 3.10+
- `uv` Python package manager installed. You can install it with:
  ```bash
  pip install uv
  ```

## 2. Setup

### a. Clone the Repository

If you haven't already, clone the project repository to your local machine.

### b. Create the Environment File

The pipeline requires API keys and configuration settings, which are loaded from a `.env` file.

1.  Navigate to the project root directory.
2.  Create a file named `.env`.
3.  Copy the contents of the example below and paste them into your `.env` file, replacing the placeholder values with your actual credentials.

**.env file contents:**
```env
# The full base URL of the deployed Docusaurus site to crawl
TARGET_SITE_URL="https://physical-ai-humanoid-robotics.com"

# Your Cohere API Key
COHERE_API_KEY="your_cohere_api_key_here"

# Your Qdrant Cloud Cluster URL
QDRANT_URL="your_qdrant_cloud_url_here"

# Your Qdrant Cloud API Key
QDRANT_API_KEY="your_qdrant_api_key_here"

# The name of the collection to create/use in Qdrant
QDRANT_COLLECTION_NAME="physical-ai-book-v1"
```

### c. Install Dependencies

The project uses `uv` to manage dependencies. A `backend` directory will be created to house the pipeline code.

1.  Open your terminal in the project root.
2.  Create the `backend` directory if it doesn't exist:
    ```bash
    mkdir backend
    ```
3.  Navigate into the backend directory:
    ```bash
    cd backend
    ```
4.  Activate the `uv` environment and install the required packages. `uv` will automatically create a virtual environment (`.venv`) inside the `backend` directory.
    ```bash
    uv pip install cohere qdrant-client beautifulsoup4 requests python-dotenv
    ```

## 3. Running the Pipeline

Once the setup is complete, you can run the ingestion pipeline.

1.  Make sure you are in the `backend` directory in your terminal.
2.  Execute the main script:
    ```bash
    uv run python main.py
    ```
3.  The script will begin execution:
    - It will fetch the sitemap from the `TARGET_SITE_URL`.
    - It will crawl each page, extract content, and create chunks.
    - It will generate embeddings for each chunk using Cohere.
    - It will upsert the embeddings and metadata to your Qdrant Cloud collection.
    - Progress and any errors will be logged to the console.

The pipeline will report a summary of its execution upon completion.
