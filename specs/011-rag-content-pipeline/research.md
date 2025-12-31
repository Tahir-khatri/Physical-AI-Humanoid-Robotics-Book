# Research: RAG Content Ingestion Pipeline

**Date**: 2025-12-30
**Feature**: `011-rag-content-pipeline`

This document summarizes the technical research and decisions for building the content ingestion pipeline.

## 1. Website Crawling Strategy

### Decision
The pipeline will crawl the target Docusaurus website by first fetching and parsing its `/sitemap.xml` file. It will then iterate through each URL found in the sitemap to process the pages.

### Rationale
- **Completeness**: Docusaurus automatically generates a `sitemap.xml` that includes all content pages, ensuring comprehensive coverage without missing unlinked "orphan" pages.
- **Efficiency**: Directly targeting listed URLs is more efficient than a recursive link-following crawler, which can get stuck in loops or miss sections of the site.
- **Simplicity**: This approach requires simple HTTP requests (`requests` library) and XML parsing (`BeautifulSoup` or Python's built-in `xml` module), avoiding the need for more complex crawling frameworks.

### Alternatives Considered
- **Recursive Link Crawling**: Following all `<a>` tags from a root URL. This is more complex to implement correctly (handling visited URLs, scope, etc.) and less efficient for a structured site like Docusaurus.

## 2. Content Extraction

### Decision
The main textual content of each page will be extracted by parsing the HTML and selecting the content within the `<main>` HTML5 tag.

### Rationale
- **Structural Consistency**: Docusaurus websites have a highly consistent and semantic HTML structure. The primary page content is reliably located within the `<main>` tag.
- **Accuracy**: This method effectively isolates the core content from boilerplate elements like headers, footers, navigation bars, and sidebars, leading to cleaner data for the embedding model.
- **Tooling**: `BeautifulSoup` is the ideal tool for this task, allowing for easy selection of elements by tag name.

## 3. Text Chunking Strategy

### Decision
A "Recursive Character Splitting" strategy will be implemented. The process will attempt to split text by a prioritized list of separators (`\n\n` for paragraphs, `\n` for lines, `.` for sentences, and finally spaces) to stay within a defined chunk size. A small overlap between chunks will be maintained.

- **Chunk Size**: ~512 tokens
- **Chunk Overlap**: ~50 tokens

### Rationale
- **Semantic Cohesion**: This method is superior to simple fixed-size chunking because it tries to keep semantically related text (paragraphs, sentences) together. This creates more meaningful chunks for the embedding model, improving retrieval relevance.
- **Robustness**: It's a widely adopted and effective baseline strategy for RAG pipelines.
- **Balance**: It provides a good balance between maintaining context and creating chunks that are small enough for efficient retrieval.

### Alternatives Considered
- **Fixed-Size Chunking**: Simpler but risks breaking sentences and semantic units, which can harm retrieval quality.
- **Sentence-Based Chunking**: Can result in highly variable chunk sizes, which is not ideal for embedding and retrieval consistency.

## 4. Technology Stack and Tooling

### Decision
The pipeline will be a Python script utilizing the following core libraries:
- **Project Management**: `uv` for creating the virtual environment and managing dependencies.
- **HTTP Requests**: `requests` for fetching the sitemap and page HTML.
- **HTML Parsing**: `beautifulsoup4` for parsing XML and HTML.
- **Embeddings**: `cohere` client library to interface with the Cohere API.
- **Vector Storage**: `qdrant-client` for batch-upserting data to Qdrant Cloud.
- **Configuration**: `python-dotenv` to manage API keys and other configuration from a `.env` file.

### Rationale
- **Industry Standard**: These libraries are the standard and recommended tools for their respective tasks in the Python ecosystem.
- **Efficiency**: `uv` provides significant speed improvements for dependency management. `qdrant-client` and `cohere` offer efficient batching capabilities.
- **Alignment**: This stack aligns perfectly with the constraints specified in the feature spec and project constitution.
