# Data Model: Physical AI & Humanoid Robotics Textbook Platform

## User Profile
- **Fields**:
  - id (string, unique, required)
  - email (string, unique, required)
  - name (string, required)
  - software_background_level (enum: beginner, intermediate, advanced, required)
  - hardware_background_level (enum: beginner, intermediate, advanced, required)
  - difficulty_preference (enum: beginner, intermediate, master, default: beginner)
  - language_preference (enum: en, ur, default: en)
  - created_at (datetime, required)
  - updated_at (datetime, required)
- **Relationships**:
  - One-to-many with UserProgress
  - One-to-many with UserInteraction

## Textbook Content
- **Fields**:
  - id (string, unique, required)
  - title (string, required)
  - module_id (string, required)
  - week_number (integer, required)
  - content_type (enum: text, video, exercise, quiz)
  - content_en (string, required)
  - content_ur (string, required)
  - difficulty_levels (object with beginner/intermediate/master content)
  - prerequisite_ids (array of strings)
  - embedding_token_size (integer, default: 150)
  - embedding_overlap_size (integer, default: 35)
  - created_at (datetime, required)
  - updated_at (datetime, required)
- **Relationships**:
  - Many-to-one with Module
  - One-to-many with ContentInteraction

## Module
- **Fields**:
  - id (string, unique, required)
  - title (string, required)
  - description (string, required)
  - module_number (integer, required)
  - weeks_count (integer, required)
  - created_at (datetime, required)
  - updated_at (datetime, required)
- **Relationships**:
  - One-to-many with TextbookContent
  - One-to-many with ModuleProgress

## User Progress
- **Fields**:
  - id (string, unique, required)
  - user_id (string, required)
  - content_id (string, required)
  - module_id (string, required)
  - completion_percentage (float, 0-100)
  - difficulty_level (enum: beginner, intermediate, master)
  - started_at (datetime, required)
  - completed_at (datetime, nullable)
  - time_spent_seconds (integer)
- **Relationships**:
  - Many-to-one with UserProfile
  - Many-to-one with TextbookContent

## Chat Query
- **Fields**:
  - id (string, unique, required)
  - user_id (string, required)
  - query_text (string, required)
  - context_text (string, nullable)
  - response_text (string, required)
  - source_documents (array of document references)
  - created_at (datetime, required)
- **Relationships**:
  - Many-to-one with UserProfile

## Content Interaction
- **Fields**:
  - id (string, unique, required)
  - user_id (string, required)
  - content_id (string, required)
  - interaction_type (enum: view, highlight, question, difficulty_change)
  - interaction_data (object, varies by type)
  - created_at (datetime, required)
- **Relationships**:
  - Many-to-one with UserProfile
  - Many-to-one with TextbookContent

## Validation Rules
- User Profile: Email must be valid format, background levels must be in enum
- Textbook Content: Module ID must reference existing module, content must exist in both languages
- Module: Module number must be unique, weeks count must be positive
- User Progress: Completion percentage must be 0-100, user and content IDs must exist
- Chat Query: Query and response text must not be empty
- Content Interaction: User and content IDs must exist, interaction type must be valid enum

## State Transitions
- User Progress: In Progress → Completed (when completion_percentage reaches 100)
- Content Interaction: New → Processed (when interaction is logged)