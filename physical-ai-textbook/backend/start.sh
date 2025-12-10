#!/bin/bash

# Startup script for RAG backend service

echo "==========================================="
echo "Physical AI & Humanoid Robotics RAG Backend"
echo "==========================================="

# Check if Python 3 is installed
if ! command -v python3 &> /dev/null; then
    echo "Error: Python 3 is not installed"
    exit 1
fi

# Check if pip is installed
if ! command -v pip3 &> /dev/null; then
    echo "Error: pip3 is not installed"
    exit 1
fi

echo "Python version: $(python3 --version)"

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
source venv/bin/activate

# Install dependencies
echo "Installing dependencies..."
pip3 install -r requirements.txt

# Check if installation was successful
if [ $? -ne 0 ]; then
    echo "Error: Failed to install dependencies"
    exit 1
fi

echo "Dependencies installed successfully!"

# Start the backend server
echo "Starting RAG backend server..."
uvicorn rag_service:app --host 0.0.0.0 --port 8000

echo "Backend server stopped."