"""
Advanced validation module for RAG retrieved chunks.

This module implements comprehensive validation checks for retrieved content chunks,
focusing on content relevance, semantic accuracy, and contextual integrity.
"""

import logging
import re
from typing import List, Dict, Any, Tuple, Optional
from sentence_transformers import SentenceTransformer
from sklearn.metrics.pairwise import cosine_similarity
import numpy as np
from .retrieval_validator import ValidationResult

# Configure logging
logger = logging.getLogger(__name__)


class ChunkValidator:
    """Advanced validator for retrieved content chunks."""

    def __init__(self):
        """Initialize the chunk validator with necessary models and configurations."""
        self.similarity_threshold = 0.3  # Minimum similarity for relevance
        self.semantic_model = None
        self._load_semantic_model()

    def _load_semantic_model(self):
        """Load the semantic similarity model."""
        try:
            # Using a lightweight model for similarity calculations
            self.semantic_model = SentenceTransformer('all-MiniLM-L6-v2')
            logger.info("Semantic model loaded successfully")
        except Exception as e:
            logger.error(f"Failed to load semantic model: {str(e)}")
            # Fallback to basic text matching if model loading fails
            self.semantic_model = None

    def validate_chunk_relevance(self, query: str, chunk: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate the relevance of a retrieved chunk to the query.

        Args:
            query: The original query string
            chunk: The retrieved chunk with content and metadata

        Returns:
            Dict: Validation results with relevance score and status
        """
        content = chunk.get('content', '')
        if not content:
            return {
                'relevance_score': 0.0,
                'is_relevant': False,
                'status': 'ERROR',
                'reason': 'Empty content in chunk'
            }

        # Calculate semantic similarity between query and content
        relevance_score = self._calculate_semantic_similarity(query, content)

        # Additional validation checks
        keyword_match_score = self._calculate_keyword_match_score(query, content)
        content_quality_score = self._assess_content_quality(content)

        # Combined relevance score (weighted average)
        combined_score = (
            0.6 * relevance_score +
            0.3 * keyword_match_score +
            0.1 * content_quality_score
        )

        is_relevant = combined_score >= self.similarity_threshold

        return {
            'relevance_score': relevance_score,
            'keyword_match_score': keyword_match_score,
            'content_quality_score': content_quality_score,
            'combined_score': combined_score,
            'is_relevant': is_relevant,
            'status': 'PASS' if is_relevant else 'FAIL',
            'reason': 'Sufficient relevance' if is_relevant else 'Low relevance score'
        }

    def validate_multiple_chunks(self, query: str, chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Validate relevance for multiple chunks.

        Args:
            query: The original query string
            chunks: List of retrieved chunks

        Returns:
            List[Dict]: Validation results for each chunk
        """
        results = []
        for chunk in chunks:
            result = self.validate_chunk_relevance(query, chunk)
            results.append(result)
        return results

    def validate_retrieval_accuracy(self, query: str, retrieved_chunks: List[Dict[str, Any]],
                                  expected_content: Optional[List[str]] = None) -> Dict[str, Any]:
        """
        Validate the overall accuracy of the retrieval.

        Args:
            query: The original query string
            retrieved_chunks: List of retrieved chunks
            expected_content: Optional list of expected content strings for comparison

        Returns:
            Dict: Overall retrieval accuracy metrics
        """
        if not retrieved_chunks:
            return {
                'accuracy_score': 0.0,
                'relevant_chunks_count': 0,
                'total_chunks': 0,
                'avg_relevance_score': 0.0,
                'coverage_score': 0.0,
                'accuracy_status': 'FAIL',
                'details': 'No chunks retrieved'
            }

        # Validate each chunk
        chunk_validations = self.validate_multiple_chunks(query, retrieved_chunks)

        # Calculate metrics
        relevant_chunks_count = sum(1 for v in chunk_validations if v['is_relevant'])
        avg_relevance_score = sum(v['combined_score'] for v in chunk_validations) / len(chunk_validations) if chunk_validations else 0
        accuracy_score = relevant_chunks_count / len(retrieved_chunks) if retrieved_chunks else 0

        # If expected content is provided, calculate coverage
        coverage_score = 0.0
        if expected_content:
            coverage_score = self._calculate_coverage_score(retrieved_chunks, expected_content)

        accuracy_status = 'PASS' if accuracy_score >= 0.5 else 'FAIL'  # 50% threshold

        return {
            'accuracy_score': accuracy_score,
            'relevant_chunks_count': relevant_chunks_count,
            'total_chunks': len(retrieved_chunks),
            'avg_relevance_score': avg_relevance_score,
            'coverage_score': coverage_score,
            'accuracy_status': accuracy_status,
            'chunk_validations': chunk_validations,
            'details': f'{relevant_chunks_count}/{len(retrieved_chunks)} chunks are relevant'
        }

    def _calculate_semantic_similarity(self, query: str, content: str) -> float:
        """Calculate semantic similarity between query and content."""
        if not self.semantic_model or not query or not content:
            # Fallback to simple overlap if semantic model is not available
            query_words = set(query.lower().split())
            content_words = set(content.lower().split())
            if not query_words:
                return 0.0
            overlap = len(query_words.intersection(content_words))
            return overlap / len(query_words) if query_words else 0.0

        try:
            # Encode both query and content
            embeddings = self.semantic_model.encode([query, content])
            # Calculate cosine similarity
            similarity = cosine_similarity([embeddings[0]], [embeddings[1]])[0][0]
            return float(similarity)
        except Exception as e:
            logger.warning(f"Semantic similarity calculation failed: {str(e)}")
            # Fallback to simple overlap
            query_words = set(query.lower().split())
            content_words = set(content.lower().split())
            if not query_words:
                return 0.0
            overlap = len(query_words.intersection(content_words))
            return overlap / len(query_words) if query_words else 0.0

    def _calculate_keyword_match_score(self, query: str, content: str) -> float:
        """Calculate keyword match score between query and content."""
        if not query or not content:
            return 0.0

        query_lower = query.lower()
        content_lower = content.lower()

        # Extract keywords from query (simple approach: non-stop words)
        query_words = set(re.findall(r'\b\w+\b', query_lower))

        # Common English stop words to filter out
        stop_words = {
            'the', 'a', 'an', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for',
            'of', 'with', 'by', 'about', 'as', 'into', 'through', 'during',
            'before', 'after', 'above', 'below', 'from', 'up', 'down', 'out',
            'is', 'are', 'was', 'were', 'be', 'been', 'being', 'have', 'has',
            'had', 'do', 'does', 'did', 'will', 'would', 'should', 'can', 'could'
        }

        query_keywords = {word for word in query_words if word not in stop_words and len(word) > 2}

        if not query_keywords:
            return 0.0

        # Find matches in content
        content_words = set(re.findall(r'\b\w+\b', content_lower))
        matches = query_keywords.intersection(content_words)

        return len(matches) / len(query_keywords) if query_keywords else 0.0

    def _assess_content_quality(self, content: str) -> float:
        """Assess the quality of the content chunk."""
        if not content:
            return 0.0

        # Check for minimum length
        if len(content.strip()) < 20:
            return 0.1  # Very low quality for short content

        # Check for readability (simple heuristic)
        words = content.split()
        avg_word_length = sum(len(word) for word in words) / len(words) if words else 0

        # Penalize content with very short or very long average word length
        if avg_word_length < 3 or avg_word_length > 12:
            quality_score = 0.3
        else:
            quality_score = 0.7

        # Check for special characters that might indicate poor quality
        special_char_ratio = len(re.findall(r'[^a-zA-Z0-9\s\.\,\!\?\;\:]', content)) / len(content) if content else 0
        if special_char_ratio > 0.1:  # More than 10% special characters
            quality_score *= 0.5

        # Check for repeated content (potential duplication)
        sentences = re.split(r'[.!?]+', content)
        unique_sentences = set(s.strip() for s in sentences if s.strip())
        if len(unique_sentences) < len(sentences) * 0.8:  # More than 20% repetition
            quality_score *= 0.7

        return min(quality_score, 1.0)

    def _calculate_coverage_score(self, retrieved_chunks: List[Dict[str, Any]],
                                expected_content: List[str]) -> float:
        """Calculate how well the retrieved chunks cover the expected content."""
        if not expected_content or not retrieved_chunks:
            return 0.0

        total_expected = len(expected_content)
        covered_count = 0

        for expected in expected_content:
            # Check if any retrieved chunk contains the expected content
            for chunk in retrieved_chunks:
                if expected.lower() in chunk.get('content', '').lower():
                    covered_count += 1
                    break

        return covered_count / total_expected if total_expected > 0 else 0.0

    def validate_chunk_contextual_integrity(self, chunk: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate the contextual integrity of a chunk.

        Args:
            chunk: The chunk to validate

        Returns:
            Dict: Validation results for contextual integrity
        """
        content = chunk.get('content', '')
        metadata = chunk.get('metadata', {})

        # Check for content coherence
        content_coherence = self._assess_content_coherence(content)

        # Check metadata consistency
        metadata_consistency = self._assess_metadata_consistency(metadata, content)

        # Check for proper chunk boundaries
        boundary_integrity = self._assess_boundary_integrity(content)

        integrity_score = (content_coherence + metadata_consistency + boundary_integrity) / 3

        return {
            'content_coherence': content_coherence,
            'metadata_consistency': metadata_consistency,
            'boundary_integrity': boundary_integrity,
            'integrity_score': integrity_score,
            'is_integrity_ok': integrity_score >= 0.6,  # 60% threshold
            'status': 'PASS' if integrity_score >= 0.6 else 'FAIL'
        }

    def _assess_content_coherence(self, content: str) -> float:
        """Assess the coherence of the content."""
        if not content:
            return 0.0

        sentences = [s.strip() for s in content.split('.') if s.strip()]
        if len(sentences) < 2:
            return 0.3  # Low coherence for single sentence

        # Check for logical flow (simple heuristic: sentence length variation)
        sentence_lengths = [len(s.split()) for s in sentences]
        if len(set(sentence_lengths)) < len(sentence_lengths) * 0.5:  # Too repetitive
            return 0.5

        # Check for complete thoughts
        complete_sentences = sum(1 for s in sentences if len(s) > 5)  # At least 5 chars
        return complete_sentences / len(sentences) if sentences else 0.0

    def _assess_metadata_consistency(self, metadata: Dict[str, Any], content: str) -> float:
        """Assess consistency between metadata and content."""
        if not metadata or not content:
            return 0.0

        content_lower = content.lower()
        consistency_score = 0.0
        total_checks = 0

        # Check if source URL is mentioned in content
        if 'source_url' in metadata:
            total_checks += 1
            url_parts = metadata['source_url'].split('/')[-1].replace('-', ' ').replace('_', ' ')
            if any(part.lower() in content_lower for part in url_parts.split() if len(part) > 3):
                consistency_score += 1

        # Check if section title is reflected in content
        if 'section_title' in metadata:
            total_checks += 1
            title_words = metadata['section_title'].lower().split()
            content_words = content_lower.split()
            title_in_content = any(word in content_words for word in title_words if len(word) > 3)
            if title_in_content:
                consistency_score += 1

        return consistency_score / total_checks if total_checks > 0 else 0.0

    def _assess_boundary_integrity(self, content: str) -> float:
        """Assess if chunk boundaries preserve semantic coherence."""
        if not content:
            return 0.0

        # Check if chunk starts/ends mid-sentence
        starts_mid_sentence = not content[0].isupper() and content[0].isalpha() if content else False
        ends_mid_sentence = content[-1] not in '.!?' if content else False

        # Simple scoring: penalize mid-sentence boundaries
        boundary_score = 1.0
        if starts_mid_sentence:
            boundary_score -= 0.3
        if ends_mid_sentence:
            boundary_score -= 0.3

        return max(boundary_score, 0.1)  # Minimum score of 0.1


def validate_retrieval_for_validation_result(validation_result: ValidationResult) -> Dict[str, Any]:
    """
    Convenience function to validate retrieval quality for a ValidationResult object.

    Args:
        validation_result: ValidationResult object to validate

    Returns:
        Dict: Comprehensive validation results
    """
    chunk_validator = ChunkValidator()

    # Extract chunks from the validation result
    chunks = []
    for result in validation_result.results:
        chunk = {
            'content': result.get('content', ''),
            'metadata': result.get('metadata', {}),
            'similarity_score': result.get('similarity_score', 0)
        }
        chunks.append(chunk)

    # Perform comprehensive validation
    retrieval_accuracy = chunk_validator.validate_retrieval_accuracy(
        validation_result.query, chunks
    )

    # Validate contextual integrity for each chunk
    contextual_validations = []
    for chunk in chunks:
        integrity_result = chunk_validator.validate_chunk_contextual_integrity(chunk)
        contextual_validations.append(integrity_result)

    return {
        'retrieval_accuracy': retrieval_accuracy,
        'contextual_validations': contextual_validations,
        'overall_validation_score': (
            retrieval_accuracy['accuracy_score'] * 0.7 +
            np.mean([v['integrity_score'] for v in contextual_validations]) * 0.3 if contextual_validations else retrieval_accuracy['accuracy_score']
        ) if chunks else 0.0
    }