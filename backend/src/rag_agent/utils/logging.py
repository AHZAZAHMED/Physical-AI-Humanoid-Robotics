"""
Logging infrastructure for the RAG agent system.
"""
import logging
import sys
import json
from datetime import datetime
from typing import Optional, Dict, Any
from dataclasses import asdict


class JSONFormatter(logging.Formatter):
    """
    Custom formatter that outputs logs in JSON format for better monitoring.
    """

    def format(self, record):
        log_entry = {
            'timestamp': datetime.utcnow().isoformat(),
            'level': record.levelname,
            'logger': record.name,
            'message': record.getMessage(),
        }

        # Add exception info if present
        if record.exc_info:
            log_entry['exception'] = self.formatException(record.exc_info)

        # Add any extra fields
        if hasattr(record, 'query_id'):
            log_entry['query_id'] = record.query_id
        if hasattr(record, 'response_time_ms'):
            log_entry['response_time_ms'] = record.response_time_ms
        if hasattr(record, 'user_id'):
            log_entry['user_id'] = record.user_id
        if hasattr(record, 'session_id'):
            log_entry['session_id'] = record.session_id

        return json.dumps(log_entry)


def setup_logging(level: str = "INFO", log_file: Optional[str] = None, json_format: bool = False):
    """
    Set up logging configuration for the application.

    Args:
        level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_file: Optional file path to write logs to
        json_format: Whether to output logs in JSON format for monitoring
    """
    # Create appropriate formatter
    if json_format:
        formatter = JSONFormatter()
    else:
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )

    # Get root logger
    logger = logging.getLogger()
    logger.setLevel(getattr(logging, level.upper()))

    # Clear existing handlers
    logger.handlers.clear()

    # Add console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    # Add file handler if specified
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    return logger


def get_logger(name: str):
    """
    Get a logger with the specified name.
    """
    return logging.getLogger(name)


def log_query_start(logger, query_text: str, query_id: str, user_id: Optional[str] = None):
    """
    Log the start of a query processing.

    Args:
        logger: The logger instance
        query_text: The query text (truncated for privacy)
        query_id: Unique query identifier
        user_id: Optional user identifier
    """
    extra = {'query_id': query_id}
    if user_id:
        extra['user_id'] = user_id

    logger.info(
        f"Starting query processing: {query_text[:100]}{'...' if len(query_text) > 100 else ''}",
        extra=extra
    )


def log_query_success(logger, query_id: str, response_time_ms: int,
                     source_count: int, grounding_confidence: float,
                     user_id: Optional[str] = None):
    """
    Log successful query completion.

    Args:
        logger: The logger instance
        query_id: Unique query identifier
        response_time_ms: Response time in milliseconds
        source_count: Number of sources used
        grounding_confidence: Grounding confidence score
        user_id: Optional user identifier
    """
    extra = {
        'query_id': query_id,
        'response_time_ms': response_time_ms
    }
    if user_id:
        extra['user_id'] = user_id

    logger.info(
        f"Query completed successfully. "
        f"Sources: {source_count}, "
        f"Grounding: {grounding_confidence:.2f}, "
        f"Time: {response_time_ms}ms",
        extra=extra
    )


def log_query_error(logger, query_id: str, error: Exception,
                   response_time_ms: int, user_id: Optional[str] = None):
    """
    Log query processing error.

    Args:
        logger: The logger instance
        query_id: Unique query identifier
        error: The error that occurred
        response_time_ms: Response time in milliseconds
        user_id: Optional user identifier
    """
    extra = {
        'query_id': query_id,
        'response_time_ms': response_time_ms
    }
    if user_id:
        extra['user_id'] = user_id

    logger.error(
        f"Query failed with error: {str(error)}",
        extra=extra,
        exc_info=True  # Include full traceback
    )


def log_cache_hit(logger, query_id: str, query_text: str):
    """
    Log cache hit for a query.

    Args:
        logger: The logger instance
        query_id: Unique query identifier
        query_text: The query text (truncated for privacy)
    """
    logger.info(
        f"Cache hit for query: {query_text[:50]}...",
        extra={'query_id': query_id}
    )


def log_cache_miss(logger, query_id: str, query_text: str):
    """
    Log cache miss for a query.

    Args:
        logger: The logger instance
        query_id: Unique query identifier
        query_text: The query text (truncated for privacy)
    """
    logger.info(
        f"Cache miss for query: {query_text[:50]}...",
        extra={'query_id': query_id}
    )


def log_cache_store(logger, query_id: str, query_text: str):
    """
    Log storing response in cache.

    Args:
        logger: The logger instance
        query_id: Unique query identifier
        query_text: The query text (truncated for privacy)
    """
    logger.info(
        f"Storing response in cache: {query_text[:50]}...",
        extra={'query_id': query_id}
    )


# Create a global logger for the application
app_logger = setup_logging(json_format=True)  # Using JSON format for better monitoring
rag_logger = get_logger("rag_agent")