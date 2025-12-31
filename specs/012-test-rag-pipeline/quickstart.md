# Quickstart: RAG Pipeline Validation Script

**Date**: 2025-12-31
**Feature**: `012-test-rag-pipeline`

This guide provides instructions on how to run the RAG pipeline validation script.

## 1. Prerequisites

- The RAG content ingestion pipeline from feature `011-rag-content-pipeline` must have been run at least once.
- A Qdrant collection must be populated with data from the ingestion pipeline.
- Python 3.10+ and `uv` should be installed.
- The `backend` directory and its dependencies must be set up as described in the quickstart for feature `011`.

## 2. Setup

### a. Environment File

The validation script uses the same `.env` file as the ingestion pipeline. Ensure the `backend/.env` file is correctly configured with the following variables:

- `TARGET_SITE_URL`
- `QDRANT_URL`
- `QDRANT_API_KEY`
- `QDRANT_COLLECTION_NAME`

The `COHERE_API_KEY` is not required for the validation script itself.

## 3. Running the Validation Script

The validation script (`retrieve.py`) will be located in the `backend` directory.

1.  Open your terminal in the `backend` directory.
2.  Execute the validation script using `uv`:
    ```bash
    uv run python retrieve.py
    ```
3.  The script will perform the following checks:
    - **Page Coverage**: Compares the URLs in the source sitemap against the URLs found in the Qdrant collection.
    - **Metadata Integrity**: Samples records to ensure all required metadata fields are present and correctly formatted.
    - **Uniqueness**: Checks for duplicate records in the collection.
4.  The script will log a detailed report of its findings to the console, including a summary of passed and failed checks, coverage percentage, and lists of any discrepancies found.

### Example Output

```
INFO - Starting RAG pipeline validation...
INFO - Fetching sitemap from https://example.com/sitemap.xml...
INFO - Found 150 URLs in sitemap.
INFO - Fetching all records from Qdrant collection 'physical-ai-book-v1'...
INFO - Retrieved 148 unique URLs from 2500 records in Qdrant.
---
Validation Report
---
[PASS] Page Coverage: 98.67% (148/150 pages found)
[PASS] Metadata Integrity: All sampled records have complete and valid metadata.
[FAIL] Uniqueness: Found 5 duplicate records. See logs for details.
---
Validation Finished
---
```
