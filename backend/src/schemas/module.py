from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime
import uuid


class ModuleBase(BaseModel):
    title: str
    module_type: str  # "ROS 2", "Gazebo/Unity", "NVIDIA Isaac", "VLA"
    description: Optional[str] = None


class ModuleCreate(ModuleBase):
    slug: str
    estimated_duration_weeks: Optional[int] = 1
    learning_objectives: Optional[List[str]] = []


class ModuleUpdate(BaseModel):
    title: Optional[str] = None
    description: Optional[str] = None
    estimated_duration_weeks: Optional[int] = None
    learning_objectives: Optional[List[str]] = None


class ModuleResponse(ModuleBase):
    id: uuid.UUID
    slug: str
    estimated_duration_weeks: int = 1
    learning_objectives: List[str] = []
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


# Update the ChapterWithModuleResponse forward reference
from pydantic import model_validator
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .chapter import ChapterResponse