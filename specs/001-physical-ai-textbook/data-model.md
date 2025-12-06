# Data Model: Physical AI & Humanoid Robotics Textbook

## Entity Definitions

### User
- **id**: UUID (Primary Key)
- **email**: String (Unique, Required)
- **username**: String (Optional)
- **password_hash**: String (Required, for authentication)
- **hardware_background**: String (Optional, enum: "beginner", "intermediate", "advanced")
- **software_background**: String (Optional, enum: "beginner", "intermediate", "advanced")
- **created_at**: DateTime (Required)
- **updated_at**: DateTime (Required)
- **is_active**: Boolean (Default: true)
- **preferences**: JSON (Optional, for personalization settings)

### Chapter
- **id**: UUID (Primary Key)
- **title**: String (Required)
- **slug**: String (Unique, Required, for URL)
- **module_id**: UUID (Foreign Key to Module, Required)
- **content**: Text (Required, Markdown/MDX format)
- **content_levels**: JSON (Optional, containing beginner/intermediate/advanced versions)
- **roman_urdu_content**: Text (Optional, translated content)
- **order**: Integer (Required, for sequence in module)
- **learning_objectives**: JSON (Optional, array of objectives)
- **practical_examples**: JSON (Optional, array of examples)
- **exercises**: JSON (Optional, array of exercises)
- **created_at**: DateTime (Required)
- **updated_at**: DateTime (Required)

### Module
- **id**: UUID (Primary Key)
- **title**: String (Required)
- **slug**: String (Unique, Required, for URL)
- **description**: Text (Optional)
- **module_type**: String (Required, enum: "ROS 2", "Gazebo/Unity", "NVIDIA Isaac", "VLA")
- **estimated_duration_weeks**: Integer (Default: 1)
- **learning_objectives**: JSON (Optional, array of objectives)
- **created_at**: DateTime (Required)
- **updated_at**: DateTime (Required)

### UserChapterProgress
- **id**: UUID (Primary Key)
- **user_id**: UUID (Foreign Key to User, Required)
- **chapter_id**: UUID (Foreign Key to Chapter, Required)
- **progress_percentage**: Float (Default: 0.0)
- **current_position**: Integer (Default: 0, character position in chapter)
- **personalization_level**: String (Optional, enum: "beginner", "intermediate", "advanced")
- **is_roman_urdu**: Boolean (Default: false)
- **last_accessed_at**: DateTime (Required)
- **completed_at**: DateTime (Optional)
- **created_at**: DateTime (Required)
- **updated_at**: DateTime (Required)

### UserPersonalizationSetting
- **id**: UUID (Primary Key)
- **user_id**: UUID (Foreign Key to User, Required)
- **chapter_id**: UUID (Foreign Key to Chapter, Optional - if null, applies globally)
- **content_depth**: String (Required, enum: "beginner", "intermediate", "advanced")
- **language_preference**: String (Required, enum: "english", "roman_urdu")
- **created_at**: DateTime (Required)
- **updated_at**: DateTime (Required)

### ChatbotConversation
- **id**: UUID (Primary Key)
- **user_id**: UUID (Foreign Key to User, Optional - null for anonymous users)
- **session_id**: UUID (Required, for grouping related queries)
- **query**: Text (Required, user's question)
- **response**: Text (Required, AI-generated answer)
- **context_used**: JSON (Optional, textbook sections referenced)
- **query_timestamp**: DateTime (Required)
- **response_timestamp**: DateTime (Required)
- **is_helpful**: Boolean (Optional, for feedback)
- **created_at**: DateTime (Required)

### TranslationCache
- **id**: UUID (Primary Key)
- **source_content_hash**: String (Unique, Required, hash of original content)
- **target_language**: String (Required, e.g., "roman_urdu")
- **translated_content**: Text (Required)
- **created_at**: DateTime (Required)
- **updated_at**: DateTime (Required)

## Relationships

- **User** 1 ←→ * **UserChapterProgress**: A user can have progress for multiple chapters
- **User** 1 ←→ * **UserPersonalizationSetting**: A user can have multiple personalization settings
- **User** 1 ←→ * **ChatbotConversation**: A user can have multiple conversations
- **Module** 1 ←→ * **Chapter**: A module contains multiple chapters
- **Chapter** 1 ←→ * **UserChapterProgress**: A chapter can have progress records from multiple users
- **Chapter** 1 ←→ * **UserPersonalizationSetting**: A chapter can have personalization settings for multiple users

## Validation Rules

### User
- Email must be valid email format
- Password must meet security requirements (min 8 chars, complexity)
- hardware_background and software_background must be one of the defined enum values

### Chapter
- Title and slug must be unique within a module
- Order must be positive integer
- Content must be in valid Markdown/MDX format

### Module
- Title and slug must be unique
- module_type must be one of: "ROS 2", "Gazebo/Unity", "NVIDIA Isaac", "VLA"

### UserChapterProgress
- progress_percentage must be between 0.0 and 100.0
- current_position must be non-negative

### UserPersonalizationSetting
- content_depth must be one of: "beginner", "intermediate", "advanced"
- language_preference must be one of: "english", "roman_urdu"

## State Transitions

### UserChapterProgress
- **Initial State**: progress_percentage = 0
- **Progress Update**: progress_percentage increases as user reads
- **Completed State**: progress_percentage = 100, completed_at timestamp set

### ChatbotConversation
- **New Query**: query received, response generated
- **Feedback Added**: is_helpful field updated based on user feedback

## Indexes for Performance

- User.email (unique)
- Chapter.slug (unique)
- Chapter.module_id + Chapter.order (for ordering within module)
- UserChapterProgress.user_id + user_chapter_id (for quick lookups)
- ChatbotConversation.session_id (for session grouping)
- TranslationCache.source_content_hash (for cache lookups)