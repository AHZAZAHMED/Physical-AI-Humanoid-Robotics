#!/bin/bash

# Load environment variables properly, handling quotes correctly
set -a
source ../.env
set +a

# Activate virtual environment and start the backend using the modular approach
source venv/bin/activate
python3 -m uvicorn src.main:app --host 0.0.0.0 --port 8000