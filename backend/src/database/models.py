from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, Float, ForeignKey, JSON, UniqueConstraint
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
import uuid

Base = declarative_base()


class User(Base):
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String, unique=True, nullable=False, index=True)
    username = Column(String, nullable=True)
    password_hash = Column(String, nullable=False)
    hardware_background = Column(String, nullable=True)  # enum: "beginner", "intermediate", "advanced"
    software_background = Column(String, nullable=True)  # enum: "beginner", "intermediate", "advanced"
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())
    is_active = Column(Boolean, default=True)
    preferences = Column(JSON, nullable=True)  # for personalization settings


class Module(Base):
    __tablename__ = "modules"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    title = Column(String, nullable=False)
    slug = Column(String, unique=True, nullable=False, index=True)
    description = Column(Text, nullable=True)
    module_type = Column(String, nullable=False)  # enum: "ROS 2", "Gazebo/Unity", "NVIDIA Isaac", "VLA"
    estimated_duration_weeks = Column(Integer, default=1)
    learning_objectives = Column(JSON, nullable=True)  # array of objectives
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())


class Chapter(Base):
    __tablename__ = "chapters"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    title = Column(String, nullable=False)
    slug = Column(String, nullable=False, index=True)  # unique within module
    module_id = Column(UUID(as_uuid=True), ForeignKey("modules.id"), nullable=False)
    content = Column(Text, nullable=False)  # Markdown/MDX format
    content_levels = Column(JSON, nullable=True)  # containing beginner/intermediate/advanced versions
    roman_urdu_content = Column(Text, nullable=True)  # translated content
    order = Column(Integer, nullable=False)  # for sequence in module
    learning_objectives = Column(JSON, nullable=True)  # array of objectives
    practical_examples = Column(JSON, nullable=True)  # array of examples
    exercises = Column(JSON, nullable=True)  # array of exercises
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Ensure slug is unique within a module
    __table_args__ = (UniqueConstraint('module_id', 'slug', name='uq_module_slug'),)


class UserChapterProgress(Base):
    __tablename__ = "user_chapter_progress"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=False, index=True)
    chapter_id = Column(UUID(as_uuid=True), ForeignKey("chapters.id"), nullable=False, index=True)
    progress_percentage = Column(Float, default=0.0)  # between 0.0 and 100.0
    current_position = Column(Integer, default=0)  # character position in chapter
    personalization_level = Column(String, nullable=True)  # enum: "beginner", "intermediate", "advanced"
    is_roman_urdu = Column(Boolean, default=False)
    last_accessed_at = Column(DateTime(timezone=True), server_default=func.now())
    completed_at = Column(DateTime(timezone=True), nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())


class UserPersonalizationSetting(Base):
    __tablename__ = "user_personalization_settings"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=False, index=True)
    chapter_id = Column(UUID(as_uuid=True), ForeignKey("chapters.id"), nullable=True, index=True)  # if null, applies globally
    content_depth = Column(String, nullable=False)  # enum: "beginner", "intermediate", "advanced"
    language_preference = Column(String, nullable=False)  # enum: "english", "roman_urdu"
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())


class ChatbotConversation(Base):
    __tablename__ = "chatbot_conversations"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=True, index=True)  # null for anonymous users
    session_id = Column(UUID(as_uuid=True), nullable=False, index=True)  # for grouping related queries
    query = Column(Text, nullable=False)  # user's question
    response = Column(Text, nullable=False)  # AI-generated answer
    context_used = Column(JSON, nullable=True)  # textbook sections referenced
    query_timestamp = Column(DateTime(timezone=True), server_default=func.now())
    response_timestamp = Column(DateTime(timezone=True), server_default=func.now())
    is_helpful = Column(Boolean, nullable=True)  # for feedback


class TranslationCache(Base):
    __tablename__ = "translation_cache"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    source_content_hash = Column(String, unique=True, nullable=False, index=True)  # hash of original content
    target_language = Column(String, nullable=False)  # e.g., "roman_urdu"
    translated_content = Column(Text, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())