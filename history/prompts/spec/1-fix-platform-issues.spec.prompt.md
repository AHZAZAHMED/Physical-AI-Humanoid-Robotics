---
id: 1
title: "Fix Platform Issues"
stage: spec
date_iso: 2025-12-16
surface: agent
model: claude-opus-4-5-20251101
feature: fix-platform-issues
branch: 1-fix-platform-issues
user: ahzaz
command: "/sp.specify"
labels: ["specification", "authentication", "chatbot", "routing", "github"]
links:
  spec: "specs/1-fix-platform-issues/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/1-fix-platform-issues/spec.md"
  - "specs/1-fix-platform-issues/checklists/requirements.md"
  - "history/prompts/spec/1-fix-platform-issues.spec.prompt.md"
tests: []
---

# Fix Platform Issues

## User Input

Please fix the following critical issues in the Physical AI Textbook platform:

1. **Complete Authentication Backend (Neon DB):**
   - The Login/Signup UI exists but is non-functional.
   - Implement the actual backend logic using Better-Auth to connect to the Neon PostgreSQL database.
   - **Registration**: Ensure that when a user registers, their email, hashed password, and background info (Hardware/Software expertise) are successfully saved to the Neon DB.
   - **Login**: Implement verification so users can log in with their saved credentials.

2. **Fix RAG Chatbot & Concurrent Server Startup:**
   - The chatbot returns a "connection error" message, likely because the backend isn't running.
   - **Action**: Create a unified startup command (e.g., in `package.json`) that runs BOTH the Docusaurus frontend (port 3000) and the FastAPI backend (port 8000) simultaneously.
   - Verify the frontend `ChatWidget` is making requests to the correct backend URL (e.g., `http://localhost:8000/chat`).

3. **Fix Routing & 404 Errors:**
   - **Current Behavior**: `.../docs/intro` returns "Page Not Found", and clicking the main title often leads to broken paths.
   - **Fix**: Adjust the Docusaurus `docusaurus.config.ts` and `sidebars.ts`:
     - Ensure the main "Introduction" file (likely `intro.mdx`) has the correct slug so it loads at `/docs/intro`.
     - Update the Navbar title link to point to the valid entry point (e.g., `/docs/intro`), preventing 404s.

4. **Update GitHub URL:**
   - Change the GitHub link in the navbar configuration to exactly: `https://github.com/AHZAZAHMED/physical-ai-textbook`

## Assistant Response

Created a comprehensive specification for fixing the critical issues in the Physical AI Textbook platform. The specification includes:

1. User stories for authentication, chatbot functionality, routing, and GitHub link updates
2. Functional requirements that are technology-agnostic
3. Success criteria with measurable outcomes
4. Quality checklist to validate the specification

The specification was created in the file `specs/1-fix-platform-issues/spec.md` with the branch name `1-fix-platform-issues`. Implementation details were removed to focus on user value and business needs.

## Outcome

Specification created successfully and is ready for the planning phase. The spec addresses all four critical issues identified by the user:
1. Authentication backend implementation
2. RAG chatbot and server startup fixes
3. Routing and 404 error resolution
4. GitHub URL update