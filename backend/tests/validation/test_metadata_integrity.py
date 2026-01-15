"""
Unit tests for RAG metadata integrity validation.

This module tests the metadata validation functionality to ensure
retrieved content chunks preserve all metadata accurately.
"""
import pytest
from backend.src.validation.metadata_validator import MetadataValidator, ValidationError


class TestMetadataIntegrity:
    """Test class for metadata integrity validation."""

    def setup_method(self):
        """Set up test fixtures before each test method."""
        self.validator = MetadataValidator()

    def test_validate_metadata_integrity_all_fields_present(self):
        """Test validation when all required fields are present."""
        chunks = [{
            'source_url': 'https://example.com/page1',
            'section_title': 'Introduction',
            'chunk_id': 'chunk_1',
            'content': 'Test content',
            'created_at': '2023-01-01',
            'author': 'Test Author'
        }]

        result = self.validator.validate_metadata_integrity(chunks)

        # Assertions
        assert result['total_chunks'] == 1
        assert result['valid_chunks'] == 1
        assert result['invalid_chunks'] == 0
        assert result['integrity_score'] == 1.0
        assert len(result['errors']) == 0

    def test_validate_metadata_integrity_missing_required_field(self):
        """Test validation when required field is missing."""
        chunks = [{
            'source_url': 'https://example.com/page1',
            # Missing 'section_title' - required field
            'chunk_id': 'chunk_1',
            'content': 'Test content',
            'created_at': '2023-01-01'
        }]

        result = self.validator.validate_metadata_integrity(chunks)

        # Assertions
        assert result['total_chunks'] == 1
        assert result['valid_chunks'] == 0
        assert result['invalid_chunks'] == 1
        assert result['integrity_score'] == 0.0
        assert len(result['errors']) > 0
        assert any('section_title' in error and 'missing' in error for error in result['errors'])

    def test_validate_metadata_integrity_empty_required_field(self):
        """Test validation when required field is empty."""
        chunks = [{
            'source_url': 'https://example.com/page1',
            'section_title': '',  # Empty required field
            'chunk_id': 'chunk_1',
            'content': 'Test content',
            'created_at': '2023-01-01'
        }]

        result = self.validator.validate_metadata_integrity(chunks)

        # Assertions
        assert result['total_chunks'] == 1
        assert result['valid_chunks'] == 0
        assert result['invalid_chunks'] == 1
        assert result['integrity_score'] == 0.0
        assert len(result['errors']) > 0
        assert any('section_title' in error and 'empty' in error for error in result['errors'])

    def test_validate_metadata_integrity_invalid_url(self):
        """Test validation when URL is invalid."""
        chunks = [{
            'source_url': 'not-a-valid-url',  # Invalid URL
            'section_title': 'Introduction',
            'chunk_id': 'chunk_1',
            'content': 'Test content',
            'created_at': '2023-01-01'
        }]

        result = self.validator.validate_metadata_integrity(chunks)

        # Assertions
        assert result['total_chunks'] == 1
        assert result['valid_chunks'] == 0
        assert result['invalid_chunks'] == 1
        assert len(result['errors']) > 0
        assert any('source_url' in error and 'Invalid URL' in error for error in result['errors'])

    def test_validate_metadata_integrity_valid_url(self):
        """Test validation when URL is valid."""
        chunks = [{
            'source_url': 'https://example.com/page1',
            'section_title': 'Introduction',
            'chunk_id': 'chunk_1',
            'content': 'Test content',
            'created_at': '2023-01-01'
        }]

        result = self.validator.validate_metadata_integrity(chunks)

        # Assertions
        assert result['total_chunks'] == 1
        assert result['valid_chunks'] == 1
        assert result['invalid_chunks'] == 0
        assert result['integrity_score'] == 1.0

    def test_validate_metadata_integrity_with_original_content_map(self):
        """Test validation with original content mapping."""
        chunks = [{
            'source_url': 'https://example.com/page1',
            'section_title': 'Introduction',
            'chunk_id': 'chunk_1',
            'content': 'Test content',
            'created_at': '2023-01-01'
        }]

        original_content_map = {
            'chunk_1': {
                'source_url': 'https://example.com/page1',
                'section_title': 'Introduction',
                'content': 'Test content'
            }
        }

        result = self.validator.validate_metadata_integrity(chunks, original_content_map)

        # Assertions
        assert result['total_chunks'] == 1
        assert result['valid_chunks'] == 1
        assert result['invalid_chunks'] == 0
        assert result['integrity_score'] == 1.0

    def test_validate_metadata_integrity_content_mismatch(self):
        """Test validation when content doesn't match original."""
        chunks = [{
            'source_url': 'https://example.com/page1',
            'section_title': 'Introduction',
            'chunk_id': 'chunk_1',
            'content': 'Different content',
            'created_at': '2023-01-01'
        }]

        original_content_map = {
            'chunk_1': {
                'source_url': 'https://example.com/page1',
                'section_title': 'Introduction',
                'content': 'Original content'
            }
        }

        result = self.validator.validate_metadata_integrity(chunks, original_content_map)

        # Assertions
        assert result['total_chunks'] == 1
        assert result['valid_chunks'] == 0
        assert result['invalid_chunks'] == 1
        assert len(result['errors']) > 0
        assert any('Content mismatch' in error for error in result['errors'])

    def test_validate_single_chunk_metadata_valid(self):
        """Test validation of a single chunk with valid metadata."""
        chunk = {
            'source_url': 'https://example.com/page1',
            'section_title': 'Introduction',
            'chunk_id': 'chunk_1',
            'content': 'Test content',
            'created_at': '2023-01-01'
        }

        errors = self.validator.validate_single_chunk_metadata(chunk)

        # Assertions
        assert len(errors) == 0

    def test_validate_single_chunk_metadata_invalid_chunk_id(self):
        """Test validation of a single chunk with invalid chunk ID."""
        chunk = {
            'source_url': 'https://example.com/page1',
            'section_title': 'Introduction',
            'chunk_id': 'invalid chunk id with spaces!',  # Invalid characters
            'content': 'Test content',
            'created_at': '2023-01-01'
        }

        errors = self.validator.validate_single_chunk_metadata(chunk)

        # Assertions
        assert len(errors) > 0
        assert any('chunk_id' in str(error) and 'Invalid chunk ID' in str(error) for error in errors)

    def test_validate_single_chunk_metadata_missing_field(self):
        """Test validation of a single chunk with missing required field."""
        chunk = {
            'source_url': 'https://example.com/page1',
            # Missing 'section_title'
            'chunk_id': 'chunk_1',
            'content': 'Test content',
            'created_at': '2023-01-01'
        }

        errors = self.validator.validate_single_chunk_metadata(chunk)

        # Assertions
        assert len(errors) > 0
        assert any('section_title' in str(error) and 'missing' in str(error) for error in errors)

    def test_field_compliance_statistics(self):
        """Test field compliance statistics calculation."""
        chunks = [
            {
                'source_url': 'https://example.com/page1',
                'section_title': 'Introduction',
                'chunk_id': 'chunk_1',
                'content': 'Test content',
                'created_at': '2023-01-01'
            },
            {
                'source_url': 'https://example.com/page2',
                'section_title': 'Conclusion',  # All fields present
                'chunk_id': 'chunk_2',
                'content': 'More content',
                'created_at': '2023-01-02'
            },
            {
                'source_url': 'https://example.com/page3',
                'section_title': 'Methods',
                'chunk_id': 'chunk_3',
                'content': 'Different content',
                # Missing 'created_at' - required field
            }
        ]

        result = self.validator.validate_metadata_integrity(chunks)

        # Assertions
        assert result['total_chunks'] == 3
        assert result['valid_chunks'] == 2  # Only first two chunks are valid
        assert result['invalid_chunks'] == 1  # Last chunk missing 'created_at'

        # Check compliance statistics
        compliance = result['field_compliance']
        assert compliance['created_at']['compliant_count'] == 2
        assert compliance['created_at']['total_count'] == 3
        assert compliance['created_at']['compliance_rate'] == 2/3