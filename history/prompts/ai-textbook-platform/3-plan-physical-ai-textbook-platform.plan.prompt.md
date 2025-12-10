---
id: 3
title: Plan Physical AI Textbook Platform
stage: plan
date: 2025-12-09
surface: agent
model: claude-opus-4-5-20251101
feature: ai-textbook-platform
branch: 1-ai-textbook-platform
user: ahzaz
command: /sp.plan
labels: [planning,architecture,ai,robotics,textbook]
links:
  spec: specs/1-ai-textbook-platform/spec.md
  plan: specs/1-ai-textbook-platform/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-ai-textbook-platform/plan.md
  - specs/1-ai-textbook-platform/research.md
  - specs/1-ai-textbook-platform/data-model.md
  - specs/1-ai-textbook-platform/contracts/openapi.yaml
  - specs/1-ai-textbook-platform/quickstart.md
tests:
---

## Prompt

# Project Plan

## Phase 1: Scaffolding & Setup
- Initialize Docusaurus project with TypeScript.
- Setup FastAPI backend structure.
- Configure TailwindCSS for Docusaurus.
- Setup Qdrant Cloud cluster and Neon Postgres database.

## Phase 2: Core Content Implementation
- Create directory structure for Modules 1-4 and Weeks 1-13.
- Generate MDX content for "Introduction" and "Hardware Requirements" based on the specification.
- Implement the Syllabus content for Weeks 1-13 in MDX.

## Phase 3: RAG Backend & Ingestion
- Develop `ingest.py`: Script to read MDX files, strip code, chunk text, embed, and upsert to Qdrant.
- Develop FastAPI Endpoint `/api/chat`: Receives query + highlighted text context.
- Implement OpenAI Agent logic: Retrieve context from Qdrant -> Generate Answer.

## Phase 4: Frontend Features (React)
- **Chat Interface:** Build Chatkit.js component interacting with FastAPI.
- **Difficulty Switcher:** Create `<DifficultySection level="beginner|intermediate|master">` component.
- **Translation:** Implement `<Translator>` wrapper or state-based text switching.
- **Auth:** Integrate Better-Auth client-side forms and protected routes.

## Phase 5: Deployment & Polish
- Run ingestion script to populate vector DB.
- Configure GitHub Actions for Pages deployment.
- Verify mobile responsiveness.

## Response snapshot

Completed the implementation planning for the Physical AI & Humanoid Robotics Textbook Platform, creating all required artifacts:

1. **Implementation Plan** (plan.md): Created comprehensive plan with technical context, constitution check, project structure, and phase completion status.

2. **Research** (research.md): Resolved all technical unknowns including technology stack selection, authentication system, content structure, language support, and difficulty level implementation.

3. **Data Model** (data-model.md): Created comprehensive data model with all required entities (User Profile, Textbook Content, Module, User Progress, Chat Query, Content Interaction) including fields, relationships, validation rules, and state transitions.

4. **API Contracts** (contracts/openapi.yaml): Generated complete OpenAPI specification for all backend endpoints including auth, content, chat, and progress tracking.

5. **Quickstart Guide** (quickstart.md): Created detailed setup guide with prerequisites, installation steps, configuration, and first development steps.

The plan aligns with all constitution principles and provides a clear roadmap for implementation following the specified phases.

## Outcome

- ✅ Impact: Complete implementation plan created with all required design artifacts
- 🧪 Tests: No functional tests required for planning artifacts
- 📁 Files: Created plan.md, research.md, data-model.md, openapi.yaml, quickstart.md
- 🔁 Next prompts: Ready for task generation (/sp.tasks) and implementation
- 🧠 Reflection: Plan addresses all aspects of the platform including content management, RAG functionality, authentication, and user experience features

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A