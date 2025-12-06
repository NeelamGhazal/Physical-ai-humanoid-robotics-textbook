from pydantic import BaseModel
from typing import Optional, List, TYPE_CHECKING
from datetime import datetime
import uuid


class ChapterBase(BaseModel):
    title: str
    module_id: uuid.UUID
    order: int


class ChapterCreate(ChapterBase):
    content: str
    slug: str
    learning_objectives: Optional[List[str]] = []
    practical_examples: Optional[List[str]] = []
    exercises: Optional[List[str]] = []


class ChapterUpdate(BaseModel):
    title: Optional[str] = None
    content: Optional[str] = None
    content_levels: Optional[dict] = None  # containing beginner/intermediate/advanced versions
    roman_urdu_content: Optional[str] = None
    learning_objectives: Optional[List[str]] = None
    practical_examples: Optional[List[str]] = None
    exercises: Optional[List[str]] = None


class ChapterResponse(ChapterBase):
    id: uuid.UUID
    slug: str
    content: str
    content_levels: Optional[dict] = None  # containing beginner/intermediate/advanced versions
    roman_urdu_content: Optional[str] = None
    learning_objectives: Optional[List[str]] = []
    practical_examples: Optional[List[str]] = []
    exercises: Optional[List[str]] = []
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class ChapterContentResponse(BaseModel):
    id: uuid.UUID
    title: str
    content: str
    module: 'ModuleResponse'
    learning_objectives: List[str] = []
    practical_examples: List[str] = []

    class Config:
        from_attributes = True


class ChapterWithModuleResponse(ChapterResponse):
    module: 'ModuleResponse'

    class Config:
        from_attributes = True


# Forward reference handling
if TYPE_CHECKING:
    from .module import ModuleResponse