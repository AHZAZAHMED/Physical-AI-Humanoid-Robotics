# Quickstart: RAG Retrieval Validation

## Overview
This guide helps you set up and run validation tests for the RAG retrieval pipeline to ensure content relevance, metadata integrity, and performance meet production standards.

## Prerequisites
- Python 3.11+
- Access to Qdrant Cloud with 'rag_embeddings' collection
- Cohere API key for generating test query embeddings
- Environment properly configured with required keys

## Setup
1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Activate the virtual environment:
   ```bash
   source .venv/bin/activate
   ```

3. Install validation dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Set up environment variables:
   ```bash
   cp .env .env.validation
   # Edit .env.validation with your Qdrant and Cohere credentials
   export $(grep -v '^#' .env.validation | xargs)
   ```

## Running Validation Tests
1. Execute the basic retrieval validation:
   ```bash
   python -m src.validation.retrieval_validator
   ```

2. Run specific validation categories:
   ```bash
   # Test keyword-based queries
   python -m src.validation.retrieval_validator --category keyword

   # Test semantic queries
   python -m src.validation.retrieval_validator --category semantic

   # Test section-specific queries
   python -m src.validation.retrieval_validator --category section-specific
   ```

3. Run comprehensive validation with performance metrics:
   ```bash
   python -m src.validation.retrieval_validator --full --concurrent 10
   ```

## Expected Output
- Validation results with PASS/FAIL status
- Performance metrics (latency, throughput)
- Relevance scores and content quality assessment
- Metadata integrity verification results
- Detailed error reports if validation fails

## Interpreting Results
- **PASS**: Retrieval meets all validation criteria
- **FAIL**: One or more validation criteria not met
- **ERROR**: Technical issue prevented validation completion

## Troubleshooting
- Ensure Qdrant collection 'rag_embeddings' exists and has content
- Verify API keys are valid and have appropriate permissions
- Check network connectivity to Qdrant and Cohere services
- Review logs for specific error details