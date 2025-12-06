from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from datetime import datetime
import uuid


class UserChapterProgressBase(BaseModel):
    user_id: uuid.UUID
    chapter_id: uuid.UUID
    progress_percentage: float = 0.0
    current_position: int = 0
    personalization_level: Optional[str] = None  # "beginner", "intermediate", "advanced"
    is_roman_urdu: bool = False


class UserChapterProgressCreate(UserChapterProgressBase):
    pass


class UserChapterProgressUpdate(BaseModel):
    progress_percentage: Optional[float] = None
    current_position: Optional[int] = None
    personalization_level: Optional[str] = None  # "beginner", "intermediate", "advanced"
    is_roman_urdu: Optional[bool] = None


class UserChapterProgressResponse(UserChapterProgressBase):
    id: uuid.UUID
    last_accessed_at: datetime
    completed_at: Optional[datetime] = None
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class UserPersonalizationSettingBase(BaseModel):
    user_id: uuid.UUID
    content_depth: str  # "beginner", "intermediate", "advanced"
    language_preference: str  # "english", "roman_urdu"


class UserPersonalizationSettingCreate(UserPersonalizationSettingBase):
    chapter_id: Optional[uuid.UUID] = None  # if null, applies globally


class UserPersonalizationSettingUpdate(BaseModel):
    content_depth: Optional[str] = None  # "beginner", "intermediate", "advanced"
    language_preference: Optional[str] = None  # "english", "roman_urdu"


class UserPersonalizationSettingResponse(UserPersonalizationSettingBase):
    id: uuid.UUID
    chapter_id: Optional[uuid.UUID] = None
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class PersonalizationSettingsResponse(BaseModel):
    global_settings: Optional[Dict[str, Any]] = None
    chapter_specific_settings: List[UserPersonalizationSettingResponse] = []

    class Config:
        from_attributes = True


class ChatbotQueryRequest(BaseModel):
    query: str
    context: Optional[str] = None  # e.g., current chapter ID


class ChatbotQueryResponse(BaseModel):
    response: str
    context_used: List[str]  # textbook sections referenced in the response
    query_id: uuid.UUID


class TranslationRequest(BaseModel):
    content: str
    target_language: str  # "roman_urdu"


class TranslationResponse(BaseModel):
    translated_content: str
    source_language: str
    target_language: str


class QuizSubmissionRequest(BaseModel):
    quiz_id: uuid.UUID
    answers: Dict[str, Any]  # question_id -> selected_answer


class QuizSubmissionResponse(BaseModel):
    score: float
    total_questions: int
    correct_answers: int
    feedback: List[Dict[str, Any]]