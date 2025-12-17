# Quickstart Guide: Physical AI Textbook Platform

**Feature**: 1-fix-platform-issues
**Created**: 2025-12-16

## Overview
This guide provides instructions for setting up and running the Physical AI Textbook platform with all the fixes from the current feature.

## Prerequisites
- Node.js (v16 or higher)
- Python (v3.8 or higher)
- PostgreSQL client tools
- Access to Neon PostgreSQL database
- npm or yarn package manager

## Setup Instructions

### 1. Clone and Navigate to Repository
```bash
git clone <repository-url>
cd physical-ai-textbook
```

### 2. Install Dependencies

#### Frontend (Docusaurus)
```bash
cd website
npm install
cd ..
```

#### Backend (FastAPI)
```bash
cd backend
pip install -r requirements.txt
cd ..
```

### 3. Environment Configuration

Create `.env` file in the backend directory with the following variables:
```env
DATABASE_URL="postgresql://username:password@ep-xxxxxx.us-east-1.aws.neon.tech/dbname"
AUTH_SECRET="your-super-secret-auth-key"
FRONTEND_URL="http://localhost:3000"
BACKEND_PORT=8000
```

### 4. Database Setup

1. Set up your Neon PostgreSQL database
2. The Better-Auth library will handle table creation automatically on first run

### 5. Running the Application

#### Unified Start (Recommended)
```bash
npm install concurrently --save-dev  # if not already installed
npm run dev  # This will start both frontend and backend simultaneously
```

#### Separate Commands
```bash
# Terminal 1: Start frontend
cd website
npm start

# Terminal 2: Start backend
cd backend
python -m uvicorn main:app --reload --port 8000
```

## Key Features Now Working

### 1. User Authentication
- Visit the registration page to create an account
- Use your email, password, and background information
- Log in with your credentials

### 2. Chatbot Functionality
- The RAG chatbot is now connected and functional
- Ask questions about Physical AI and Humanoid Robotics
- The chatbot will respond with information from the textbook content

### 3. Proper Routing
- Navigate to `/docs/intro` to access the introduction page
- All navbar links work correctly
- No more 404 errors on main documentation pages

### 4. GitHub Link
- The navbar GitHub link now correctly points to: https://github.com/AHZAZAHMED/physical-ai-textbook

## Troubleshooting

### Common Issues

#### Port Already in Use
- Check if another process is using port 3000 or 8000
- Kill the process or use different ports

#### Database Connection Issues
- Verify your Neon database credentials in the `.env` file
- Ensure the database is accessible from your network
- Check that the database URL is properly formatted

#### Authentication Not Working
- Ensure Better-Auth is properly configured with your database
- Check that the auth secret is set in the environment
- Verify that the frontend is making requests to the correct backend URL

#### Chatbot Connection Errors
- Verify that the backend server is running
- Check that the frontend is configured to connect to the correct backend endpoint
- Confirm that CORS settings allow frontend-backend communication

## Next Steps

1. Explore the documentation pages
2. Try out the chatbot with questions about Physical AI
3. Register an account to access personalized features
4. Contribute to the project via the GitHub repository