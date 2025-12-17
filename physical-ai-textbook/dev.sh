#!/bin/bash

# Development startup script for Physical AI Textbook platform
# Runs both frontend (Docusaurus) and backend (FastAPI) servers

echo "==========================================="
echo "Physical AI & Humanoid Robotics Platform"
echo "Starting Frontend and Backend Development Servers"
echo "==========================================="

# Function to handle cleanup on exit
cleanup() {
    echo "Shutting down servers..."
    kill %1 %2 2>/dev/null
    exit 0
}

# Set up trap to catch signals and clean up properly
trap cleanup INT TERM

# Start the backend server in the background
echo "Starting backend server on port 8000..."
cd backend && source venv/bin/activate && python -m uvicorn rag_service:app --host 0.0.0.0 --port 8000 &
BACKEND_PID=$!

# Small delay to let backend start
sleep 3

# Start the frontend server in the background
echo "Starting frontend server on port 3000..."
cd ../ && npm start &
FRONTEND_PID=$!

# Wait for both processes
wait $BACKEND_PID $FRONTEND_PID