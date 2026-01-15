"""
Unit tests for RAG chunk validation functionality.

This module tests the advanced chunk validation functionality focusing on
content relevance, semantic accuracy, and contextual integrity.
"""
import pytest
from backend.src.validation.chunk_validator import ChunkValidator


class TestChunkValidation:
    """Test class for chunk validation functionality."""

    def setup_method(self):
        """Set up test fixtures before each test method."""
        self.validator = ChunkValidator()

    def test_validate_chunk_relevance_basic(self):
        """Test basic chunk relevance validation."""
        query = "artificial intelligence in robotics"
        chunk = {
            'content': "Artificial intelligence plays a crucial role in modern robotics applications.",
            'metadata': {}
        }

        result = self.validator.validate_chunk_relevance(query, chunk)

        # Assertions
        assert 'relevance_score' in result
        assert 'is_relevant' in result
        assert 'status' in result
        assert result['is_relevant']  # Should be relevant based on content
        assert result['status'] in ['PASS', 'FAIL']

    def test_validate_chunk_relevance_empty_content(self):
        """Test chunk relevance validation with empty content."""
        query = "artificial intelligence"
        chunk = {
            'content': "",  # Empty content
            'metadata': {}
        }

        result = self.validator.validate_chunk_relevance(query, chunk)

        # Assertions
        assert result['relevance_score'] == 0.0
        assert result['is_relevant'] is False
        assert result['status'] == 'ERROR'
        assert 'Empty content' in result['reason']

    def test_validate_multiple_chunks(self):
        """Test validation of multiple chunks."""
        query = "machine learning algorithms"
        chunks = [
            {
                'content': "Machine learning algorithms are used in various applications.",
                'metadata': {}
            },
            {
                'content': "Deep learning is a subset of machine learning.",
                'metadata': {}
            },
            {
                'content': "This content is not related to machine learning.",
                'metadata': {}
            }
        ]

        results = self.validator.validate_multiple_chunks(query, chunks)

        # Assertions
        assert len(results) == 3
        for result in results:
            assert 'relevance_score' in result
            assert 'is_relevant' in result

    def test_validate_retrieval_accuracy_no_chunks(self):
        """Test retrieval accuracy validation with no chunks."""
        query = "test query"
        retrieved_chunks = []

        result = self.validator.validate_retrieval_accuracy(query, retrieved_chunks)

        # Assertions
        assert result['accuracy_score'] == 0.0
        assert result['relevant_chunks_count'] == 0
        assert result['total_chunks'] == 0
        assert result['accuracy_status'] == 'FAIL'

    def test_validate_retrieval_accuracy_with_chunks(self):
        """Test retrieval accuracy validation with relevant chunks."""
        query = "robotics applications"
        retrieved_chunks = [
            {
                'content': "Robotics applications include manufacturing and healthcare.",
                'metadata': {}
            },
            {
                'content': "Modern robotics uses various sensors and actuators.",
                'metadata': {}
            }
        ]

        result = self.validator.validate_retrieval_accuracy(query, retrieved_chunks)

        # Assertions
        assert 'accuracy_score' in result
        assert 'relevant_chunks_count' in result
        assert 'total_chunks' in result
        assert result['total_chunks'] == 2
        assert 0 <= result['accuracy_score'] <= 1.0

    def test_calculate_keyword_match_score(self):
        """Test keyword match score calculation."""
        query = "artificial intelligence machine learning"
        content = "Artificial intelligence and machine learning are transforming industries."

        score = self.validator._calculate_keyword_match_score(query, content)

        # Assertions
        assert 0 <= score <= 1.0
        assert score > 0.5  # Should have good keyword match

    def test_calculate_keyword_match_score_no_match(self):
        """Test keyword match score with no match."""
        query = "quantum computing"
        content = "Cooking recipes for Italian cuisine."

        score = self.validator._calculate_keyword_match_score(query, content)

        # Assertions
        assert 0 <= score <= 1.0
        assert score < 0.1  # Should have very low match

    def test_assess_content_quality_good(self):
        """Test content quality assessment for good content."""
        content = "Artificial intelligence is a branch of computer science that aims to create software or machines that exhibit human-like intelligence."

        score = self.validator._assess_content_quality(content)

        # Assertions
        assert 0 <= score <= 1.0
        assert score > 0.5  # Should be good quality

    def test_assess_content_quality_poor(self):
        """Test content quality assessment for poor content."""
        content = "abc 123 xyz"  # Very short, low quality content

        score = self.validator._assess_content_quality(content)

        # Assertions
        assert 0 <= score <= 1.0
        assert score < 0.3  # Should be low quality

    def test_validate_chunk_contextual_integrity(self):
        """Test contextual integrity validation."""
        chunk = {
            'content': "Artificial intelligence (AI) is intelligence demonstrated by machines.",
            'metadata': {
                'source_url': 'https://example.com/ai-intro',
                'section_title': 'Introduction to AI'
            }
        }

        result = self.validator.validate_chunk_contextual_integrity(chunk)

        # Assertions
        assert 'content_coherence' in result
        assert 'metadata_consistency' in result
        assert 'boundary_integrity' in result
        assert 'integrity_score' in result
        assert 0 <= result['integrity_score'] <= 1.0

    def test_assess_content_coherence(self):
        """Test content coherence assessment."""
        content = "Artificial intelligence is important. It has many applications in robotics."

        score = self.validator._assess_content_coherence(content)

        # Assertions
        assert 0 <= score <= 1.0

    def test_assess_metadata_consistency(self):
        """Test metadata consistency assessment."""
        metadata = {
            'source_url': 'https://example.com/robotics-fundamentals',
            'section_title': 'Robotics Fundamentals'
        }
        content = "Robotics Fundamentals cover kinematics and control systems."

        score = self.validator._assess_metadata_consistency(metadata, content)

        # Assertions
        assert 0 <= score <= 1.0

    def test_assess_boundary_integrity_good(self):
        """Test boundary integrity assessment for good chunk."""
        content = "Artificial intelligence is a wonderful field. It encompasses many subfields."

        score = self.validator._assess_boundary_integrity(content)

        # Assertions
        assert 0 <= score <= 1.0
        assert score >= 0.7  # Should be good boundary integrity

    def test_assess_boundary_integrity_poor(self):
        """Test boundary integrity assessment for poor chunk."""
        content = "artificial intelligence is a wonderful field that has many applications in robotics and other areas"  # No ending punctuation

        score = self.validator._assess_boundary_integrity(content)

        # Assertions
        assert 0 <= score <= 1.0
        # Score should be lower due to missing ending punctuation