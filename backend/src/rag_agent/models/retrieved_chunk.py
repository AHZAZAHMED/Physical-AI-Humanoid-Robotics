"""
RetrievedChunk model representing a book content chunk retrieved from the vector database.
"""
from dataclasses import dataclass
from typing import Dict, Any, Optional


@dataclass
class RetrievedChunk:
    """
    Represents a book content chunk retrieved from the vector database.
    """
    id: str
    content: str
    similarity_score: float
    confidence_score: float
    metadata: Dict[str, Any]
    vector_id: Optional[str] = None
    embedding: Optional[list] = None

    def validate(self) -> bool:
        """
        Validate the retrieved chunk parameters.
        """
        if not self.content or len(self.content.strip()) == 0:
            raise ValueError("Content must not be empty")

        if not (0.0 <= self.similarity_score <= 1.0):
            raise ValueError("Similarity score must be between 0.0 and 1.0")

        if not (0.0 <= self.confidence_score <= 1.0):
            raise ValueError("Confidence score must be between 0.0 and 1.0")

        required_metadata = ['book_title', 'chapter', 'section']
        for field in required_metadata:
            if field not in self.metadata:
                raise ValueError(f"Metadata must contain '{field}' field")

        return True