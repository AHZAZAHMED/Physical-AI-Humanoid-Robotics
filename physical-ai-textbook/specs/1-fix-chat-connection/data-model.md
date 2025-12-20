# Data Model: Fix Chat Service Connection Issues

## Entities

### 1. Chat Connection
- **Description**: Represents the communication channel between frontend and backend services
- **Attributes**:
  - connection_id (string): Unique identifier for the connection
  - status (string): Current status (connected, disconnected, error)
  - created_at (datetime): Timestamp when connection was established
  - last_activity (datetime): Timestamp of last activity
  - error_message (string, optional): Error message if connection failed

### 2. Chat Message
- **Description**: Represents a single message in the chat conversation
- **Attributes**:
  - message_id (string): Unique identifier for the message
  - session_id (string): Identifier for the chat session
  - sender_type (string): Type of sender (user, bot)
  - content (string): The actual message content
  - timestamp (datetime): When the message was sent
  - sources (array of strings, optional): Source documents referenced in response

### 3. Chat Session
- **Description**: Represents a conversation session between user and chatbot
- **Attributes**:
  - session_id (string): Unique identifier for the session
  - user_id (string, optional): Associated user ID (null for anonymous sessions)
  - created_at (datetime): When the session was created
  - last_accessed (datetime): When the session was last used
  - is_active (boolean): Whether the session is currently active

### 4. Backend Service
- **Description**: The FastAPI server hosting the chat endpoint
- **Attributes**:
  - service_url (string): Base URL of the service (e.g., http://localhost:8000)
  - endpoint_path (string): Specific endpoint path (/chat)
  - status (string): Current status (running, unavailable, error)
  - health_check_url (string): Health check endpoint URL

### 5. Frontend Client
- **Description**: The Docusaurus-based frontend application
- **Attributes**:
  - client_type (string): Type of client (web browser)
  - connection_config (object): Configuration for connecting to backend
  - retry_config (object): Retry logic configuration
  - error_handling_config (object): Error handling configuration

## Relationships

### Chat Session and Chat Messages
- One Chat Session contains many Chat Messages (1 to many)
- Messages are associated with a session via session_id

### Chat Connection and Chat Session
- One Chat Connection can be associated with one or more Chat Sessions
- Sessions can be created or accessed through an active connection

### Backend Service and Chat Connection
- Backend Service receives requests from multiple Chat Connections
- Each connection attempts to communicate with the backend service

## State Transitions

### Chat Connection States
- `disconnected` → `connecting` → `connected` (successful connection)
- `disconnected` → `connecting` → `error` (connection failed)
- `connected` → `disconnected` (connection lost)
- `error` → `connecting` (retry after error)

### Session States
- `inactive` → `active` (when user starts chatting)
- `active` → `inactive` (when session times out or user closes chat)

## Validation Rules

### Chat Message Validation
- Content must not be empty
- Content length should be reasonable (max 10000 characters)
- Session ID must exist and be active

### Chat Connection Validation
- Service URL must be a valid URL format
- Connection timeout should be between 5-30 seconds
- Retry attempts should be limited (max 5 attempts)

### Session Validation
- Session must be created before messages can be added
- Session must be active to accept new messages
- Anonymous sessions should have time-based expiration