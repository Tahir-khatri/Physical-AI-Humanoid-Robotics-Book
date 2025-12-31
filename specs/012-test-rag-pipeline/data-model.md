# Data Model: RAG Pipeline Validation Script

**Date**: 2025-12-31
**Feature**: `012-test-rag-pipeline`

This validation script does not introduce a new data model. It consumes and validates the data model defined by the ingestion pipeline feature (`011-rag-content-pipeline`).

## Consumed Entity: `VectorRecord`

The script will retrieve and inspect records from Qdrant, which are expected to conform to the `VectorRecord` payload schema.

### Payload Schema (for reference)

The `payload` object within each Qdrant point is expected to contain the following fields:

- **`text`**: `string`
  - The actual text content of the chunk.
- **`url`**: `string`
  - The source URL from which the chunk was extracted.
- **`section_title`**: `string`
  - The nearest preceding heading to the chunk.
- **`chunk_index`**: `integer`
  - The 0-based index of the chunk on the page.
- **`processed_at`**: `string` (ISO 8601 format)
  - The timestamp of when the chunk was processed.

The validation script will verify the presence and basic format of these fields in the retrieved records.
