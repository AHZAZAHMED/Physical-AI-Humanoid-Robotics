#!/bin/bash

# Startup script for Physical AI & Humanoid Robotics Textbook

echo "==========================================="
echo "Physical AI & Humanoid Robotics Textbook"
echo "==========================================="

# Check if Node.js is installed
if ! command -v node &> /dev/null; then
    echo "Error: Node.js is not installed"
    exit 1
fi

# Check if npm is installed
if ! command -v npm &> /dev/null; then
    echo "Error: npm is not installed"
    exit 1
fi

echo "Node.js version: $(node --version)"
echo "npm version: $(npm --version)"

# Install dependencies
echo "Installing dependencies..."
npm install

# Check if installation was successful
if [ $? -ne 0 ]; then
    echo "Error: Failed to install dependencies"
    exit 1
fi

echo "Dependencies installed successfully!"

# Start the development server in the background
echo "Starting development server..."
npm start &

# Get the process ID of the background job
SERVER_PID=$!

echo "Development server started with PID: $SERVER_PID"
echo "Textbook is available at: http://localhost:3000"
echo ""
echo "To stop the server, run: kill $SERVER_PID"
echo ""
echo "For backend API (in another terminal):"
echo "cd backend && python3 -m pip install -r requirements.txt"
echo "python3 rag_service.py"

# Wait for user input to stop the server
echo ""
echo "Press any key to stop the server..."
read -n 1 -s

# Kill the server process
kill $SERVER_PID
echo ""
echo "Server stopped."