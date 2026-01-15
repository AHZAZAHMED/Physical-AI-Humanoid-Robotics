---
description: "Task list for Phase-1 Docusaurus Landing Page & Documentation Routing implementation"
---

# Tasks: Phase-1 Docusaurus Landing Page & Documentation Routing

**Input**: Design documents from `/specs/2-docusaurus-landing/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus Project**: `src/`, `docs/`, `sidebars.js`, `docusaurus.config.js`
- **Pages**: `src/pages/`
- **Theme Components**: `src/theme/`
- **Configuration**: `docusaurus.config.js`, `sidebars.js`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Analyze current Docusaurus configuration in docusaurus.config.js
- [x] T002 Inspect existing documentation structure in docs/ directory
- [x] T003 [P] Review sidebar configuration in sidebars.js
- [x] T004 Identify current redirect configuration causing root URL issue

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

Foundational tasks for this feature:

- [x] T005 Remove redirect plugin configuration from root to /docs/intro in docusaurus.config.js
- [x] T006 [P] Verify documentation remains accessible at /docs path after redirect removal
- [x] T007 [P] Confirm base URL configuration in docusaurus.config.js is correct for GitHub Pages

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Access Root Landing Page (Priority: P1) 🎯 MVP

**Goal**: Create an engaging landing page at the root URL (/) instead of showing a "Page Not Found" error

**Independent Test**: The landing page can be tested by visiting the root URL and verifying that it displays the introductory content and call-to-action button without showing any errors.

### Implementation for User Story 1

- [x] T008 [P] Create landing page component in src/pages/index.tsx
- [x] T009 [US1] Add book title "Physical AI & Humanoid Robotics" to landing page
- [x] T010 [US1] Add tagline "Embodied Intelligence: Bridging the Digital Brain and the Physical Body" to landing page
- [x] T011 [US1] Add brief introduction explaining the book's content to landing page
- [x] T012 [US1] Add explanation of the modular structure of the textbook to landing page
- [x] T013 [US1] Add prominent call-to-action button linking to /docs on landing page
- [x] T014 [US1] Ensure landing page renders correctly at root URL (/) with responsive design

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Navigate to Book Content (Priority: P1)

**Goal**: Ensure book content is accessible at the /docs URL path so users can easily find and read the textbook materials without confusion

**Independent Test**: The book content can be tested by navigating to /docs and verifying that the documentation pages load correctly.

### Implementation for User Story 2

- [x] T015 [P] Verify documentation is served at /docs path after redirect removal
- [x] T016 [US2] Confirm intro document is correctly mapped as the entry page at /docs
- [x] T017 [US2] Test that /docs opens book content correctly without nested routing
- [x] T018 [US2] Verify call-to-action button from landing page navigates to /docs correctly

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Avoid Nested Routing Issues (Priority: P2)

**Goal**: Prevent seeing nested URLs like /docs/docs/intro to keep navigation clean and intuitive

**Independent Test**: The routing can be tested by checking that book content is accessed directly at /docs without nested paths like /docs/docs.

### Implementation for User Story 3

- [x] T019 [P] Inspect current docs directory structure for potential nesting issues
- [x] T020 [US3] Remove or refactor any redundant nesting that could cause /docs/docs/... pattern
- [x] T021 [US3] Configure Docusaurus docs to use /docs as the base route without nesting
- [x] T022 [US3] Verify all documentation URLs follow /docs/page-name pattern, not /docs/docs/page-name

**Checkpoint**: At this point, User Stories 1, 2 AND 3 should all work independently

---

## Phase 6: User Story 4 - Maintain Navigation Functionality (Priority: P1)

**Goal**: Ensure sidebar and navigation function correctly throughout the site so users can easily move between different sections of the content

**Independent Test**: Navigation functionality can be tested by clicking through sidebar items and verifying that they lead to the correct content pages.

### Implementation for User Story 4

- [x] T023 [P] Validate sidebar configuration against updated documentation routes in sidebars.js
- [x] T024 [US4] Ensure sidebar loads correctly when accessing /docs
- [x] T025 [US4] Verify navbar includes link to /docs in docusaurus.config.js
- [x] T026 [US4] Confirm consistent navigation between landing page and documentation
- [x] T027 [US4] Test navigation functionality across different pages and sections

**Checkpoint**: All user stories should now be independently functional

---

## Phase 7: Design Enhancement (Professional Landing Page)

**Purpose**: Enhance the landing page to look professional with good design, not just plain text

- [x] T036 [P] [US1] Add professional styling and visual design elements to src/pages/index.tsx
- [x] T037 [US1] Implement attractive color scheme and typography in src/css/custom.css
- [x] T038 [US1] Add visual elements like icons, graphics, or images to enhance the landing page
- [x] T039 [US1] Improve layout spacing and visual hierarchy in src/pages/index.tsx
- [x] T040 [US1] Create a more engaging hero section with better visual appeal
- [x] T041 [US1] Enhance the call-to-action button with professional styling
- [x] T042 [US1] Add visual elements that represent the book's content (ROS, Simulation, etc.)
- [x] T043 [US1] Implement responsive design improvements for all screen sizes
- [x] T044 [US1] Add animations or interactive elements to improve user experience

---
## Phase 8: Testing and Verification

**Purpose**: Comprehensive testing of all routing and navigation functionality

- [x] T045 [P] Test root URL loads the professionally designed landing page correctly
- [x] T046 [P] Test /docs opens the book content correctly
- [x] T047 Identify and fix any 404 or routing errors
- [x] T048 [P] Perform a production build to validate deployment readiness
- [x] T049 Test deep links to documentation pages
- [x] T050 Verify behavior across different browsers and devices
- [x] T051 [P] Validate base URL configuration matches project deployment path
- [x] T052 Ensure all internal links respect the base URL

---

## Phase 9: Phase Completion Review

**Purpose**: Final validation and completion of Phase-1 routing and UX setup

- [x] T056 Confirm all success criteria from the plan are met
- [x] T057 Ensure no unintended routes or broken links remain
- [x] T058 Mark Phase-1 routing and UX setup as complete

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Design Enhancement (Phase 7)**: Depends on all desired user stories being complete
- **Testing and Verification (Phase 8)**: Depends on all desired user stories being complete
- **Phase Completion Review (Phase 9)**: Depends on all previous phases being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - May depend on US1 (landing page)
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 4 (P1)**: Can start after Foundational (Phase 2) - May depend on US1 and US2

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all parallel tasks for User Story 1 together:
Task: "Create landing page component in src/pages/index.tsx"
Task: "Add book title 'Physical AI & Humanoid Robotics' to landing page"
Task: "Add tagline 'Embodied Intelligence: Bridging the Digital Brain and the Physical Body' to landing page"
```

---

## Implementation Strategy

### MVP First (User Stories 1, 2, and 4 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Landing Page)
4. Complete Phase 4: User Story 2 (Documentation Access)
5. Complete Phase 6: User Story 4 (Navigation)
6. **STOP and VALIDATE**: Test core functionality independently
7. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (Landing page!)
3. Add User Story 2 → Test independently → Deploy/Demo (Docs access!)
4. Add User Story 4 → Test independently → Deploy/Demo (Navigation!)
5. Add User Story 3 → Test independently → Deploy/Demo (Routing fix!)
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Landing Page)
   - Developer B: User Story 2 (Documentation Access)
   - Developer C: User Story 4 (Navigation)
   - Developer D: User Story 3 (Routing Fix)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [US1], [US2], [US3], [US4] labels map task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence