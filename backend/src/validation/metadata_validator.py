"""
Metadata validation module for RAG retrieval validation system.

This module validates that retrieved content chunks preserve all metadata accurately,
including source URLs, section titles, and other metadata fields.
"""

import logging
from typing import List, Dict, Any, Optional
from .retrieval_validator import ValidationResult

# Configure logging
logger = logging.getLogger(__name__)


class ValidationError:
    """Details about specific validation failures."""

    def __init__(self, field: str, expected: Any, actual: Any, message: str):
        self.field = field
        self.expected = expected
        self.actual = actual
        self.message = message

    def __str__(self):
        return f"{self.field}: {self.message} (expected: {self.expected}, actual: {self.actual})"


class MetadataValidator:
    """Validates metadata integrity in retrieved content chunks."""

    def __init__(self):
        self.required_fields = [
            'source_url',
            'section_title',
            'chunk_id',
            'content',
            'created_at'
        ]
        self.optional_fields = [
            'author',
            'tags',
            'category',
            'page_number',
            'word_count'
        ]

    def validate_metadata_integrity(self, retrieved_chunks: List[Dict],
                                  original_content_map: Optional[Dict] = None) -> Dict[str, Any]:
        """
        Validate metadata integrity for retrieved chunks.

        Args:
            retrieved_chunks: List of retrieved content chunks with metadata
            original_content_map: Optional mapping of chunk IDs to original content for comparison

        Returns:
            Dict: Validation results with errors and statistics
        """
        validation_results = {
            'total_chunks': len(retrieved_chunks),
            'valid_chunks': 0,
            'invalid_chunks': 0,
            'errors': [],
            'field_compliance': {},
            'integrity_score': 0.0
        }

        for i, chunk in enumerate(retrieved_chunks):
            chunk_errors = []

            # Validate required fields exist
            for field in self.required_fields:
                if field not in chunk:
                    error = ValidationError(
                        field=field,
                        expected="present",
                        actual="missing",
                        message=f"Required field '{field}' is missing from chunk metadata"
                    )
                    chunk_errors.append(error)
                elif chunk[field] is None or chunk[field] == '':
                    error = ValidationError(
                        field=field,
                        expected="non-empty",
                        actual=chunk[field],
                        message=f"Required field '{field}' is empty or None"
                    )
                    chunk_errors.append(error)

            # Validate content integrity if original content map is provided
            if original_content_map and 'chunk_id' in chunk:
                chunk_id = chunk['chunk_id']
                if chunk_id in original_content_map:
                    original_content = original_content_map[chunk_id]
                    if original_content.get('content', '') != chunk.get('content', ''):
                        error = ValidationError(
                            field='content',
                            expected=original_content.get('content', ''),
                            actual=chunk.get('content', ''),
                            message=f"Content mismatch for chunk {chunk_id}"
                        )
                        chunk_errors.append(error)

                    # Validate other fields against original
                    for field in ['source_url', 'section_title']:
                        if field in original_content and field in chunk:
                            if original_content[field] != chunk[field]:
                                error = ValidationError(
                                    field=field,
                                    expected=original_content[field],
                                    actual=chunk[field],
                                    message=f"Metadata field '{field}' mismatch for chunk {chunk_id}"
                                )
                                chunk_errors.append(error)

            # Validate URL format if present
            if 'source_url' in chunk:
                url = chunk['source_url']
                if not self._is_valid_url(url):
                    error = ValidationError(
                        field='source_url',
                        expected="valid URL format",
                        actual=url,
                        message=f"Invalid URL format: {url}"
                    )
                    chunk_errors.append(error)

            # Validate chunk ID format
            if 'chunk_id' in chunk:
                chunk_id = chunk['chunk_id']
                if not self._is_valid_chunk_id(chunk_id):
                    error = ValidationError(
                        field='chunk_id',
                        expected="valid chunk ID format",
                        actual=chunk_id,
                        message=f"Invalid chunk ID format: {chunk_id}"
                    )
                    chunk_errors.append(error)

            # Update results
            if chunk_errors:
                validation_results['invalid_chunks'] += 1
                validation_results['errors'].extend([
                    f"Chunk {i}: {str(error)}" for error in chunk_errors
                ])
            else:
                validation_results['valid_chunks'] += 1

        # Calculate field compliance
        for field in self.required_fields:
            compliant_count = sum(1 for chunk in retrieved_chunks
                                if field in chunk and chunk[field] is not None and chunk[field] != '')
            compliance_rate = compliant_count / len(retrieved_chunks) if retrieved_chunks else 0
            validation_results['field_compliance'][field] = {
                'compliant_count': compliant_count,
                'total_count': len(retrieved_chunks),
                'compliance_rate': compliance_rate
            }

        # Calculate overall integrity score
        validation_results['integrity_score'] = (
            validation_results['valid_chunks'] / len(retrieved_chunks) if retrieved_chunks else 0
        )

        return validation_results

    def validate_single_chunk_metadata(self, chunk: Dict, chunk_id: str = None) -> List[ValidationError]:
        """
        Validate metadata for a single chunk.

        Args:
            chunk: Single content chunk with metadata
            chunk_id: Optional chunk ID for error reporting

        Returns:
            List[ValidationError]: List of validation errors found
        """
        errors = []

        # Validate required fields
        for field in self.required_fields:
            if field not in chunk:
                error = ValidationError(
                    field=field,
                    expected="present",
                    actual="missing",
                    message=f"Required field '{field}' is missing"
                )
                errors.append(error)
            elif chunk[field] is None or chunk[field] == '':
                error = ValidationError(
                    field=field,
                    expected="non-empty",
                    actual=chunk[field],
                    message=f"Required field '{field}' is empty or None"
                )
                errors.append(error)

        # Validate URL format if present
        if 'source_url' in chunk:
            url = chunk['source_url']
            if not self._is_valid_url(url):
                error = ValidationError(
                    field='source_url',
                    expected="valid URL format",
                    actual=url,
                    message=f"Invalid URL format: {url}"
                )
                errors.append(error)

        # Validate chunk ID if provided or in chunk
        chunk_id_to_check = chunk_id or chunk.get('chunk_id')
        if chunk_id_to_check and not self._is_valid_chunk_id(chunk_id_to_check):
            error = ValidationError(
                field='chunk_id',
                expected="valid chunk ID format",
                actual=chunk_id_to_check,
                message=f"Invalid chunk ID format: {chunk_id_to_check}"
            )
            errors.append(error)

        return errors

    def _is_valid_url(self, url: str) -> bool:
        """Check if a URL is valid."""
        if not url or not isinstance(url, str):
            return False

        # Simple URL validation - check for basic structure
        import re
        url_pattern = re.compile(
            r'^https?://'  # http:// or https://
            r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # domain...
            r'localhost|'  # localhost...
            r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
            r'(?::\d+)?'  # optional port
            r'(?:/?|[/?]\S+)$', re.IGNORECASE)

        return url_pattern.match(url) is not None

    def _is_valid_chunk_id(self, chunk_id: str) -> bool:
        """Check if a chunk ID is valid."""
        if not chunk_id or not isinstance(chunk_id, str):
            return False

        # Check if it's a valid format (alphanumeric, hyphens, underscores)
        import re
        id_pattern = re.compile(r'^[a-zA-Z0-9_-]+$')
        return id_pattern.match(chunk_id) is not None

    def validate_retrieval_result_metadata(self, validation_result: ValidationResult) -> Dict[str, Any]:
        """
        Validate metadata for an entire validation result.

        Args:
            validation_result: ValidationResult object to validate

        Returns:
            Dict: Metadata validation results
        """
        # Extract chunks from the validation result
        chunks = []
        for result in validation_result.results:
            if 'metadata' in result:
                chunk = result['metadata'].copy()
                chunk['content'] = result.get('content', '')
                chunks.append(chunk)

        return self.validate_metadata_integrity(chunks)


def validate_metadata_for_retrieval_results(validation_results: List[ValidationResult]) -> Dict[str, Any]:
    """
    Convenience function to validate metadata for multiple validation results.

    Args:
        validation_results: List of ValidationResult objects

    Returns:
        Dict: Combined metadata validation results
    """
    metadata_validator = MetadataValidator()

    all_chunks = []
    for result in validation_results:
        for res in result.results:
            if 'metadata' in res:
                chunk = res['metadata'].copy()
                chunk['content'] = res.get('content', '')
                all_chunks.append(chunk)

    return metadata_validator.validate_metadata_integrity(all_chunks)