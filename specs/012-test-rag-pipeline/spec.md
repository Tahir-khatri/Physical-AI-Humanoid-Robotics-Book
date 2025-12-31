# Feature Specification: Test RAG Pipeline

**Feature Branch**: `012-test-rag-pipeline`  
**Created**: 2025-12-31
**Status**: Draft  
**Input**: User description: "Test and validate URL ingestion & embedding pipeline for RAG chatbot Target audience: Developers ensuring RAG backend reliability for the project book Focus: Verify that all crawled pages are processed, embeddings are correctly generated, and vectors + metadata are properly stored in Qdrant Cloud Success criteria: - Confirms ≥95% page coverage - Ensures all chunks have correct metadata (URL, heading, chunk index, timestamp) - Validates embeddings are stored in Qdrant Cloud without duplication - Confirms pipeline rerun does not overwrite or duplicate data - Logs processed and failed pages for debugging Constraints: - Use Python + Cohere embeddings + Qdrant Cloud Free Tier - Test only deployed Docusaurus GitHub Pages URLs - CLI/script-based execution - Modular and and config-driven code Not building: - Retrieval logic or chatbot UI - Frontend integration - Model evaluation workflows - Analytics or dashboard features"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Verify Page Coverage and Metadata (Priority: P1)

A developer needs to confirm that the ingestion pipeline successfully processed all expected pages and that the extracted metadata is correct.

**Why this priority**: This is fundamental to ensuring the RAG system has accurate and complete information.

**Independent Test**: Can be tested by running the validation script against a Qdrant collection populated by the ingestion pipeline and comparing results against expected sitemap content.

**Acceptance Scenarios**:

1. **Given** a Qdrant collection populated by the RAG ingestion pipeline and access to the original Docusaurus sitemap.
2. **When** the validation script is executed.
3. **Then** the script reports ≥95% page coverage and confirms the presence of correct metadata (URL, heading, chunk index, timestamp) for a random sample of records.

---

### User Story 2 - Validate Embedding Integrity and Uniqueness (Priority: P2)

A developer needs to ensure that embeddings are correctly generated and stored in Qdrant without duplication.

**Why this priority**: Correct embeddings are crucial for retrieval accuracy, and duplicates waste resources and can skew results.

**Independent Test**: Can be tested by generating a small dataset, processing it with the ingestion pipeline, and then running the validation script to check for embedding consistency and the absence of duplicates in Qdrant.

**Acceptance Scenarios**:

1. **Given** a Qdrant collection populated by the RAG ingestion pipeline.
2. **When** the validation script analyzes the embeddings.
3. **Then** the script confirms that embeddings adhere to expected dimensions and no identical vectors (representing duplicate content) are stored.

---

### User Story 3 - Confirm Idempotent Pipeline Rerun (Priority: P2)

A developer needs to verify that re-running the ingestion pipeline does not lead to data corruption, loss, or excessive duplication.

**Why this priority**: Ensures the pipeline can be reliably operated and re-executed without adverse effects on the vector store.

**Independent Test**: Can be tested by running the ingestion pipeline twice on the same content, then executing the validation script to ensure the final state of the Qdrant collection is consistent and free from unintended changes.

**Acceptance Scenarios**:

1. **Given** a Qdrant collection successfully populated by an initial ingestion pipeline run.
2. **When** the ingestion pipeline is re-run on the same content.
3. **Then** the validation script reports that no data was corrupted or lost, and no new, identical duplicates were added.

### Edge Cases
- What happens if the Qdrant collection is empty or does not exist? The script should report an error and terminate gracefully.
- What happens if the Docusaurus sitemap URL is invalid or inaccessible? The script should report an error and terminate.
- How does the script handle pages that are listed in the sitemap but could not be processed during ingestion (e.g., due to errors)? The script should identify these discrepancies.
- What if metadata fields are missing or malformed for some records in Qdrant? The script should flag these inconsistencies.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The system MUST connect to the specified Qdrant Cloud instance using provided credentials.
- **FR-002**: The system MUST retrieve the URLs from the original Docusaurus sitemap.
- **FR-003**: The system MUST compare the URLs found in the sitemap with the URLs associated with records in the Qdrant collection to calculate page coverage.
- **FR-004**: The system MUST sample a configurable number of records from the Qdrant collection and validate that their payloads contain all required metadata fields (URL, section title, chunk index, timestamp).
- **FR-005**: The system MUST detect and report records in Qdrant where essential metadata fields are missing or malformed.
- **FR-006**: The system MUST identify and report instances of exact duplicate vectors (same embedding and payload) within the Qdrant collection.
- **FR-007**: The system MUST log detailed information about validation successes, failures, and discrepancies.
- **FR-008**: The validation process MUST be executable via a Command Line Interface (CLI).
- **FR-009**: All external service credentials and configuration (Qdrant URL, API key, collection name, sitemap URL) MUST be configurable via environment variables or a configuration file.

### Key Entities
- **Qdrant Collection**: The target vector database to be validated.
- **Sitemap**: The XML file listing all expected URLs from the source Docusaurus site.
- **Vector Record**: An entry in the Qdrant collection. (Attributes: vector embedding, payload).
- **Metadata**: Attributes within a Vector Record's payload: URL, Section Title, Chunk Index, Timestamp.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The validation script consistently reports ≥95% page coverage between the Docusaurus sitemap and Qdrant collection.
- **SC-002**: The validation script confirms that all sampled vectors possess complete and correctly formatted metadata fields (URL, section title, chunk index, timestamp).
- **SC-003**: The validation script accurately identifies and reports any duplicate vector entries in the Qdrant collection.
- **SC-004**: After re-running the ingestion pipeline, the validation script confirms that the Qdrant collection's state is consistent (no unintended duplication or loss) with the expected outcome.
- **SC-005**: The validation script's logs clearly differentiate between successful validations, warnings (e.g., minor discrepancies), and critical errors (e.g., missing essential data, major coverage gaps).