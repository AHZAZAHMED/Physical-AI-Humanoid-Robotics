# Feature Specification: Fix Platform Issues

**Feature Branch**: `1-fix-platform-issues`
**Created**: 2025-12-16
**Status**: Draft
**Input**: User description: "Please fix the following critical issues in the Physical AI Textbook platform:

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
   - Change the GitHub link in the navbar configuration to exactly: `https://github.com/AHZAZAHMED/physical-ai-textbook`"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Enable User Registration and Login (Priority: P1)

Users need to be able to create accounts and log in to access the Physical AI Textbook platform. Currently, the UI exists but authentication is non-functional, preventing users from creating personalized accounts.

**Why this priority**: This is fundamental to user engagement and access to personalized features of the platform.

**Independent Test**: Users can successfully register with an email, password, and background information, then log in with those credentials.

**Acceptance Scenarios**:

1. **Given** a new user visits the platform, **When** they navigate to the registration page and provide valid email, password, and background info, **Then** they can successfully create an account and see a confirmation message
2. **Given** a user has an existing account, **When** they navigate to the login page and provide valid credentials, **Then** they can successfully log in and access the platform
3. **Given** a user provides invalid credentials, **When** they attempt to log in, **Then** they receive an appropriate error message and remain on the login page

---

### User Story 2 - Enable Chatbot Functionality (Priority: P1)

Users need to interact with the RAG chatbot to get information about Physical AI and Humanoid Robotics content. Currently, the chatbot shows connection errors because the backend server isn't properly running or connected.

**Why this priority**: The chatbot is a core feature for user engagement and learning support.

**Independent Test**: Users can interact with the chatbot, submit questions, and receive responses without connection errors.

**Acceptance Scenarios**:

1. **Given** the platform is running, **When** a user submits a question to the chatbot widget, **Then** they receive a relevant response within a reasonable time
2. **Given** the platform is running, **When** the frontend and backend servers are both active, **Then** the chatbot functions without connection errors
3. **Given** the platform is running, **When** a user refreshes the page, **Then** the chatbot remains accessible and functional

---

### User Story 3 - Fix Navigation and Routing (Priority: P2)

Users need to navigate through the documentation content without encountering 404 errors. Currently, key pages like `/docs/intro` return "Page Not Found" and the main title link leads to broken paths.

**Why this priority**: Proper navigation is essential for users to access and consume the educational content.

**Independent Test**: Users can access the introduction page at `/docs/intro` and navigate from the main title link without 404 errors.

**Acceptance Scenarios**:

1. **Given** a user navigates to `/docs/intro`, **When** they access the URL, **Then** the introduction page loads successfully without a 404 error
2. **Given** a user clicks the main title in the navbar, **When** they initiate the navigation, **Then** they are directed to the appropriate introduction page
3. **Given** a user is browsing documentation, **When** they use sidebar navigation, **Then** all links lead to valid pages

---

### User Story 4 - Update GitHub Link (Priority: P3)

Users need to access the correct GitHub repository from the platform. Currently, the GitHub link in the navbar may point to an incorrect URL.

**Why this priority**: Provides users with access to the source code and development resources.

**Independent Test**: Users can click the GitHub link in the navbar and be directed to the correct repository URL.

**Acceptance Scenarios**:

1. **Given** a user is on the platform, **When** they click the GitHub link in the navbar, **Then** they are redirected to `https://github.com/AHZAZAHMED/physical-ai-textbook`
2. **Given** the GitHub link is displayed in the navbar, **When** the page loads, **Then** the link contains the correct URL

---

### Edge Cases

- What happens when a user tries to register with an email that already exists?
- How does the system handle backend server failures when the chatbot is being used?
- What occurs when the database connection fails during authentication?
- How does the system behave when a user attempts to access a non-existent documentation page that's not the main intro page?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST implement secure backend authentication logic that connects to a database
- **FR-002**: System MUST allow users to register with email, password, and background information (Hardware/Software expertise) that gets securely stored in the database
- **FR-003**: System MUST allow users to log in with their stored credentials and verify them against the database
- **FR-004**: System MUST run both frontend and backend servers simultaneously using a unified startup command
- **FR-005**: System MUST enable the chatbot widget to communicate with the backend API server
- **FR-006**: System MUST ensure the introduction page is accessible at `/docs/intro` without 404 errors
- **FR-007**: System MUST update the navbar title link to point to the correct entry point
- **FR-008**: System MUST update the GitHub link in the navbar to `https://github.com/AHZAZAHMED/physical-ai-textbook`
- **FR-009**: System MUST handle authentication errors gracefully with appropriate user feedback
- **FR-010**: System MUST maintain session state for authenticated users

### Key Entities *(include if feature involves data)*

- **User**: Represents a registered user with attributes: email, hashed password, background information (Hardware/Software expertise), creation date
- **Session**: Represents an active user session with attributes: user ID, session token, expiration time

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can successfully register and log in with 95% success rate
- **SC-002**: The chatbot responds to user queries without connection errors 99% of the time during normal operation
- **SC-003**: All documentation pages accessible via navigation return 200 status codes, with less than 1% 404 errors
- **SC-004**: The GitHub link in the navbar correctly redirects to the specified repository URL 100% of the time
- **SC-005**: Both frontend and backend services start simultaneously with a single command within a reasonable time
- **SC-006**: Users report 90% satisfaction with platform navigation and access to content