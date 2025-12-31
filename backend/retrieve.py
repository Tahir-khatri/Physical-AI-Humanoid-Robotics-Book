import os
import logging
import requests
from bs4 import BeautifulSoup
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv
from typing import List, Dict, Any
from collections import Counter

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def load_config() -> dict:
    """Loads environment variables from a .env file and returns them as a dictionary."""
    load_dotenv()
    config = {
        "TARGET_SITE_URL": os.getenv("TARGET_SITE_URL"),
        "QDRANT_URL": os.getenv("QDRANT_URL"),
        "QDRANT_API_KEY": os.getenv("QDRANT_API_KEY"),
        "QDRANT_COLLECTION_NAME": os.getenv("QDRANT_COLLECTION_NAME", "physical-ai-book-v1"),
    }
    # Basic validation
    for key, value in config.items():
        if value is None:
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
    logging.info(f"Fetching sitemap from {sitemap_url}...")
    response = requests.get(sitemap_url)
    response.raise_for_status()
    soup = BeautifulSoup(response.content, 'xml')
    urls = [loc.text for loc in soup.find_all('loc')]
    logging.info(f"Found {len(urls)} URLs in sitemap.")
    return urls

def get_all_qdrant_records(config: dict) -> List[models.Record]:
    """
    Fetches all records from the specified Qdrant collection using the scroll API.

    Args:
        config (dict): Configuration dictionary with Qdrant connection details.

    Returns:
        List[models.Record]: A list of all records found in the collection.
    """
    client = QdrantClient(
        url=config["QDRANT_URL"],
        api_key=config["QDRANT_API_KEY"],
    )
    logging.info(f"Fetching all records from Qdrant collection '{config['QDRANT_COLLECTION_NAME']}'...")
    
    all_records = []
    next_offset = None
    while True:
        records, next_offset = client.scroll(
            collection_name=config["QDRANT_COLLECTION_NAME"],
            limit=250,
            with_payload=True,
            with_vectors=True,
            offset=next_offset
        )
        all_records.extend(records)
        if next_offset is None:
            break
            
    logging.info(f"Retrieved {len(all_records)} records from Qdrant.")
    return all_records

def validate_page_coverage(sitemap_urls: List[str], qdrant_records: List[models.Record]) -> bool:
    """
    Validates that the pages in Qdrant cover at least 95% of the pages in the sitemap.
    """
    qdrant_urls = {record.payload['url'] for record in qdrant_records}
    sitemap_url_set = set(sitemap_urls)
    
    missing_urls = sitemap_url_set - qdrant_urls
    if len(sitemap_url_set) == 0:
        logging.error("Sitemap is empty, cannot calculate coverage.")
        return False
        
    coverage = (len(sitemap_url_set) - len(missing_urls)) / len(sitemap_url_set) * 100
    
    logging.info(f"Page Coverage: {coverage:.2f}% ({len(sitemap_url_set) - len(missing_urls)}/{len(sitemap_url_set)} pages found)")
    
    if missing_urls:
        logging.warning(f"Missing URLs: {missing_urls}")
        
    return coverage >= 95

def validate_metadata(qdrant_records: List[models.Record], sample_size: int = 20) -> bool:
    """
    Validates the metadata of a sample of records from the Qdrant collection.
    """
    if not qdrant_records:
        logging.warning("No records in Qdrant to validate metadata against.")
        return True # Or False, depending on desired strictness

    sample_records = qdrant_records[:sample_size]
    required_keys = {"url", "section_title", "chunk_index", "processed_at"}
    all_valid = True

    for record in sample_records:
        payload = record.payload
        missing_keys = required_keys - set(payload.keys())
        if missing_keys:
            logging.error(f"Record ID {record.id} is missing metadata keys: {missing_keys}")
            all_valid = False
            
        if len(record.vector) != 1024:
            logging.error(f"Record ID {record.id} has incorrect vector dimension: {len(record.vector)}")
            all_valid = False

    if all_valid:
        logging.info("Metadata and vector dimension validation passed for all sampled records.")
    
    return all_valid

def validate_uniqueness(qdrant_records: List[models.Record]) -> bool:
    """
    Validates that there are no duplicate records in the Qdrant collection.
    """
    seen_identifiers = set()
    duplicates = []
    
    for record in qdrant_records:
        identifier = (record.payload['url'], record.payload.get('chunk_index')) # .get() for safety
        if identifier in seen_identifiers:
            duplicates.append(identifier)
        else:
            seen_identifiers.add(identifier)
            
    if duplicates:
        logging.error(f"Found {len(duplicates)} duplicate records: {duplicates}")
        return False
    else:
        logging.info("Uniqueness validation passed. No duplicate records found.")
        return True

def main():
    """
    Main function to run the entire validation pipeline.
    """
    try:
        config = load_config()
        sitemap_urls = get_sitemap_urls(config["TARGET_SITE_URL"])
        qdrant_records = get_all_qdrant_records(config)
        
        logging.info("--- Validation Report ---")
        
        coverage_passed = validate_page_coverage(sitemap_urls, qdrant_records)
        metadata_passed = validate_metadata(qdrant_records)
        uniqueness_passed = validate_uniqueness(qdrant_records)
        
        logging.info("--- Validation Finished ---")
        
        print("\n--- Validation Summary ---")
        print(f"Page Coverage Check: {'PASS' if coverage_passed else 'FAIL'}")
        print(f"Metadata Integrity Check: {'PASS' if metadata_passed else 'FAIL'}")
        print(f"Uniqueness Check: {'PASS' if uniqueness_passed else 'FAIL'}")
        print("------------------------")

        if all([coverage_passed, metadata_passed, uniqueness_passed]):
            logging.info("All validation checks passed successfully.")
        else:
            logging.error("One or more validation checks failed.")

    except Exception as e:
        logging.error(f"An unexpected error occurred during validation: {e}")

if __name__ == "__main__":
    main()
