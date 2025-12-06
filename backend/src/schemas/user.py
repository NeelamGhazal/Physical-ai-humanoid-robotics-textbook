from pydantic import BaseModel, EmailStr, field_validator
from typing import Optional, List
from datetime import datetime
import uuid


class UserBase(BaseModel):
    email: EmailStr
    username: Optional[str] = None


class UserCreate(UserBase):
    password: str
    hardware_background: Optional[str] = None  # "beginner", "intermediate", "advanced"
    software_background: Optional[str] = None  # "beginner", "intermediate", "advanced"

    @field_validator('hardware_background', 'software_background')
    @classmethod
    def validate_background_level(cls, v):
        if v and v not in ["beginner", "intermediate", "advanced"]:
            raise ValueError('Background level must be one of: beginner, intermediate, advanced')
        return v


class UserUpdate(BaseModel):
    username: Optional[str] = None
    hardware_background: Optional[str] = None  # "beginner", "intermediate", "advanced"
    software_background: Optional[str] = None  # "beginner", "intermediate", "advanced"

    @field_validator('hardware_background', 'software_background')
    @classmethod
    def validate_background_level(cls, v):
        if v and v not in ["beginner", "intermediate", "advanced"]:
            raise ValueError('Background level must be one of: beginner, intermediate, advanced')
        return v


class UserResponse(UserBase):
    id: uuid.UUID
    hardware_background: Optional[str] = None  # "beginner", "intermediate", "advanced"
    software_background: Optional[str] = None  # "beginner", "intermediate", "advanced"
    is_active: bool = True
    created_at: datetime

    class Config:
        from_attributes = True


class UserLogin(BaseModel):
    email: EmailStr
    password: str


class TokenResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"
    user: UserResponse