# Implementation Tasks: Fix Platform Issues

**Feature**: 1-fix-platform-issues
**Created**: 2025-12-16
**Status**: Active
**Branch**: 1-fix-platform-issues

## Overview

This document outlines the implementation tasks for fixing critical issues in the Physical AI Textbook platform:
1. Foundational Backend: Implement Better-Auth and connect to Neon Database, set up Qdrant for content embedding
2. Critical Bug Fixes: Fix Docusaurus routing and GitHub navbar link
3. Dev Environment: Create concurrent startup script

## Dependencies

User stories can be implemented in parallel after foundational setup is complete:
- US1 (Authentication) and US2 (Chatbot) require backend setup (T001-T012)
- US3 (Routing) and US4 (GitHub Link) can be implemented independently

## Parallel Execution Examples

- Backend setup (T001-T012) can run in parallel with frontend fixes (T020-T024)
- Authentication (US1) and Chatbot (US2) can be developed in parallel after foundational setup
- Routing fix and GitHub link fix can be done simultaneously

## Implementation Strategy

Start with foundational setup (Phase 1-2) to enable all user stories, then implement user stories in priority order. MVP includes successful authentication and login functionality (US1).

---

## Phase 1: Setup

### Goal
Initialize project structure and install required dependencies

- [X] T001 Set up project structure with backend and frontend directories
- [X] T002 Install authentication and PostgreSQL dependencies in backend
- [X] T003 Install Qdrant client and related dependencies for content embedding
- [X] T004 Install concurrently package for unified startup
- [X] T005 Configure environment variables for Neon database connection
- [X] T006 Set up initial backend FastAPI application structure

## Phase 2: Foundational Backend

### Goal
Implement core backend infrastructure needed for all user stories

- [X] T010 [P] Implement authentication configuration with PostgreSQL support in backend/src/auth/config.py
- [X] T011 [P] Configure custom user schema with background info field in backend/src/auth/models.py
- [X] T012 [P] Set up Qdrant vector database and create ingestion script for book content in backend/src/rag/ingest.py
- [X] T013 [P] Create database connection pool for Neon PostgreSQL in backend/src/db/connection.py
- [X] T014 [P] Implement user session management in backend/src/auth/session.py
- [X] T015 [P] Set up API middleware for authentication in backend/src/middleware/auth.py
- [X] T016 [P] Create health check endpoint at backend/src/api/health.py
- [X] T017 [P] Configure CORS settings to allow frontend communication in backend/main.py

## Phase 3: User Story 1 - Enable User Registration and Login (Priority: P1)

### Goal
Enable users to create accounts and log in to access the Physical AI Textbook platform

### Independent Test Criteria
Users can successfully register with an email, password, and background information, then log in with those credentials

- [X] T020 [P] [US1] Create registration endpoint /auth/register in backend/src/api/auth.py
- [X] T021 [P] [US1] Create login endpoint /auth/login in backend/src/api/auth.py
- [X] T022 [P] [US1] Create logout endpoint /auth/logout in backend/src/api/auth.py
- [X] T023 [P] [US1] Create user profile endpoint /auth/me in backend/src/api/auth.py
- [X] T024 [P] [US1] Implement email validation and uniqueness checks in backend/src/services/user_service.py
- [X] T025 [P] [US1] Implement proper error handling for authentication failures in backend/src/api/auth.py
- [X] T026 [P] [US1] Test user data persistence in Neon DB with registration and login
- [X] T027 [P] [US1] Create API tests for authentication endpoints in backend/tests/test_auth.py

## Phase 4: User Story 2 - Enable Chatbot Functionality (Priority: P1)

### Goal
Enable users to interact with the RAG chatbot to get information about Physical AI and Humanoid Robotics content

### Independent Test Criteria
Users can interact with the chatbot, submit questions, and receive responses without connection errors

- [X] T030 [P] [US2] Create chat endpoint /chat in backend/src/api/chat.py
- [X] T031 [P] [US2] Implement RAG logic to retrieve relevant content from Qdrant in backend/src/rag/retriever.py
- [X] T032 [P] [US2] Create chat service to handle conversation logic in backend/src/services/chat_service.py
- [X] T033 [P] [US2] Implement chat history management in backend/src/services/chat_service.py
- [X] T034 [P] [US2] Create health check endpoint /chat/health in backend/src/api/chat.py
- [X] T035 [P] [US2] Test chatbot functionality with sample queries in backend/tests/test_chat.py
- [X] T036 [P] [US2] Verify frontend ChatWidget connects to backend API at http://localhost:8000/chat in src/theme/Chatbot/index.js

## Phase 5: User Story 3 - Fix Navigation and Routing (Priority: P2)

### Goal
Users can navigate through the documentation content without encountering 404 errors

### Independent Test Criteria
Users can access the introduction page at `/docs/intro` and navigate from the main title link without 404 errors

- [X] T040 [P] [US3] Update intro.mdx with proper slug configuration at physical-ai-textbook/docs/intro.md
- [X] T041 [P] [US3] Update docusaurus.config.ts to fix navbar title link to point to intro page at physical-ai-textbook/docusaurus.config.js
- [X] T042 [P] [US3] Verify sidebar navigation works properly in physical-ai-textbook/sidebars.js
- [X] T043 [P] [US3] Test all navigation paths for 404 errors
- [X] T044 [P] [US3] Ensure localhost:3000/ redirects correctly to the proper entry point in docusaurus.config.js

## Phase 6: User Story 4 - Update GitHub Link (Priority: P3)

### Goal
Users can access the correct GitHub repository from the platform

### Independent Test Criteria
Users can click the GitHub link in the navbar and be directed to the correct repository URL

- [X] T050 [P] [US4] Locate navbar configuration in docusaurus.config.js and update GitHub link
- [X] T051 [P] [US4] Update GitHub link to exactly https://github.com/AHZAZAHMED/physical-ai-textbook in docusaurus.config.js
- [X] T052 [P] [US4] Verify link works correctly from all pages in the application

## Phase 7: Dev Environment

### Goal
Create unified startup for frontend and backend services

- [X] T060 [P] Update package.json with unified start script using concurrently
- [X] T061 [P] Create start:frontend and start:backend scripts in package.json
- [X] T062 [P] Test simultaneous startup of both services with no port conflicts
- [X] T063 [P] Document the unified startup process in README.md

## Phase 8: Integration & Testing

### Goal
Test complete user flows across all fixed components

- [X] T070 [P] Test end-to-end user registration and login flow
- [X] T071 [P] Verify chatbot functionality with unified startup
- [X] T072 [P] Test all navigation paths work without 404 errors
- [X] T073 [P] Verify GitHub link works from all pages
- [X] T074 [P] Validate startup time is reasonable with concurrent processes
- [X] T075 [P] Test concurrent user sessions and database connection efficiency

## Phase 9: Polish & Cross-Cutting Concerns

### Goal
Final quality assurance and documentation

- [X] T080 Update documentation to reflect new authentication flow
- [X] T081 Ensure all error responses follow consistent format
- [X] T082 Add logging for authentication and chat interactions
- [X] T083 Perform final testing of all user stories
- [X] T084 Update project README with new setup instructions