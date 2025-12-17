# Data Model: Physical AI Textbook Platform

**Feature**: 1-fix-platform-issues
**Created**: 2025-12-16
**Status**: Draft

## Overview
This document defines the data models for the Physical AI Textbook platform, focusing on user authentication and profile management.

## Entity: User

### Attributes
- **id** (string): Unique identifier for the user (UUID or database-generated)
- **email** (string): User's email address (unique, required, valid email format)
- **password** (string): Hashed password (managed by Better-Auth, required)
- **backgroundInfo** (string): User's background information including hardware/software expertise (optional, text field)
- **createdAt** (timestamp): Account creation date and time
- **updatedAt** (timestamp): Last account update date and time
- **emailVerified** (boolean): Whether the user's email has been verified (default: false)
- **isActive** (boolean): Whether the account is active (default: true)

### Relationships
- **Session**: One-to-many (one user can have multiple active sessions)
- **Preferences**: One-to-one (user preferences, if implemented later)

### Validation Rules
- Email must be unique across all users
- Email must match standard email format
- Password must meet security requirements (handled by Better-Auth)
- Background info, if provided, should be less than 1000 characters

### State Transitions
- **Unverified** → **Verified**: When user completes email verification
- **Active** → **Inactive**: When account is deactivated by user or admin

## Entity: Session

### Attributes
- **id** (string): Unique identifier for the session (UUID)
- **userId** (string): Reference to the user who owns this session
- **token** (string): Secure session token (managed by Better-Auth)
- **expiresAt** (timestamp): Session expiration time
- **createdAt** (timestamp): Session creation time
- **lastAccessedAt** (timestamp): Last time the session was used
- **deviceInfo** (string): Information about the device used (optional)
- **ipAddress** (string): IP address of the session origin (optional, for security)

### Relationships
- **User**: Many-to-one (many sessions belong to one user)

### Validation Rules
- Session tokens must be cryptographically secure
- Sessions must expire after a defined period of inactivity
- Multiple concurrent sessions per user are allowed

## Entity: UserProfile (Extension)

### Attributes
- **userId** (string): Reference to the user (primary key)
- **displayName** (string): User's display name (optional)
- **bio** (string): Brief biography or description (optional)
- **learningGoals** (json): User's learning goals in Physical AI and robotics (optional)
- **createdAt** (timestamp): Profile creation time
- **updatedAt** (timestamp): Last profile update time

### Relationships
- **User**: One-to-one (extends user information)

### Validation Rules
- displayName, if present, should be 3-50 characters
- bio, if present, should be less than 500 characters
- learningGoals, if present, should be valid JSON

## Database Schema Considerations

### Indexes
- Index on `users.email` for fast login lookups
- Index on `sessions.userId` for session management
- Index on `sessions.expiresAt` for cleanup operations

### Security
- Passwords are never stored in plain text
- Session tokens are securely generated and stored
- Personal information access is protected by authentication

## API Data Contracts

### Registration Request
```
{
  "email": "user@example.com",
  "password": "securePassword123",
  "backgroundInfo": "Software engineer with robotics experience"
}
```

### Registration Response
```
{
  "id": "user-uuid",
  "email": "user@example.com",
  "backgroundInfo": "Software engineer with robotics experience",
  "createdAt": "2025-12-16T10:00:00Z",
  "emailVerified": false
}
```

### Login Request
```
{
  "email": "user@example.com",
  "password": "securePassword123"
}
```

### Login Response
```
{
  "user": {
    "id": "user-uuid",
    "email": "user@example.com",
    "backgroundInfo": "Software engineer with robotics experience"
  },
  "session": {
    "token": "session-token",
    "expiresAt": "2025-12-17T10:00:00Z"
  }
}
```

## Future Extensibility
- Additional user preference fields
- Integration with learning progress tracking
- Social features and user connections
- Content creation and sharing capabilities