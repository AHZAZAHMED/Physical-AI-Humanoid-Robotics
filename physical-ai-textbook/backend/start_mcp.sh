#!/bin/bash

# Script to start the Qdrant MCP server
echo "Starting Qdrant MCP server..."

# Set environment variables for the MCP server
export QDRANT_URL="https://dda1fdab-d1f4-4b10-9483-77fc01aa9c55.us-east4-0.gcp.cloud.qdrant.io:6333"
export QDRANT_API_KEY="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.pcX4lcFHbGFNIeivUHS02-Va9npeQ2n0stttogk5k4s"
export COLLECTION_NAME="book"
export EMBEDDING_MODEL="sentence-transformers/all-MiniLM-L6-v2"

echo "QDRANT_URL: $QDRANT_URL"
echo "COLLECTION_NAME: $COLLECTION_NAME"
echo "EMBEDDING_MODEL: $EMBEDDING_MODEL"

# Run the Qdrant MCP server with stdio transport
uvx mcp-server-qdrant --transport stdio