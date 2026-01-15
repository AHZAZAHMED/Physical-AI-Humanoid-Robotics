"""
Query model representing a user's input query and associated parameters.
"""
from dataclasses import dataclass
from typing import Dict, Any, Optional
from datetime import datetime


@dataclass
class Query:
    """
    Represents a user's input query and associated parameters.
    """
    id: str
    text: str
    embedding: Optional[list] = None
    parameters: Optional[Dict[str, Any]] = None
    timestamp: datetime = None
    user_id: Optional[str] = None

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now()

    def validate(self) -> bool:
        """
        Validate the query parameters.
        """
        if not self.text or len(self.text.strip()) == 0:
            raise ValueError("Query text must not be empty")

        if len(self.text) > 1000:
            raise ValueError("Query text must be less than 1000 characters")

        if self.parameters:
            top_k = self.parameters.get('top_k', 5)
            if not (1 <= top_k <= 20):
                raise ValueError("top_k must be between 1 and 20")

            confidence_threshold = self.parameters.get('confidence_threshold', 0.7)
            if not (0.0 <= confidence_threshold <= 1.0):
                raise ValueError("confidence_threshold must be between 0.0 and 1.0")

        return True