from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy.exc import IntegrityError
from typing import Optional
import uuid
import jwt
from datetime import datetime, timedelta
from passlib.context import CryptContext

from backend.src.database.models import User
from backend.src.schemas.user import UserCreate, UserResponse, UserLogin
from backend.src.config import settings


# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


class UserService:
    @staticmethod
    def verify_password(plain_password: str, hashed_password: str) -> bool:
        """Verify a plain password against a hashed password"""
        return pwd_context.verify(plain_password, hashed_password)

    @staticmethod
    def get_password_hash(password: str) -> str:
        """Hash a password"""
        return pwd_context.hash(password)

    @staticmethod
    async def get_user_by_id(db: AsyncSession, user_id: str) -> Optional[UserResponse]:
        """Get a user by their ID"""
        try:
            uuid_obj = uuid.UUID(user_id)
        except ValueError:
            return None

        result = await db.execute(select(User).where(User.id == uuid_obj))
        user = result.scalar_one_or_none()

        if user:
            return UserResponse.from_orm(user)
        return None

    @staticmethod
    async def get_user_by_email(db: AsyncSession, email: str) -> Optional[UserResponse]:
        """Get a user by their email address"""
        result = await db.execute(select(User).where(User.email == email))
        user = result.scalar_one_or_none()

        if user:
            return UserResponse.from_orm(user)
        return None

    @staticmethod
    async def create_user(db: AsyncSession, user_create: UserCreate) -> UserResponse:
        """Create a new user in the database"""
        # Hash the password
        hashed_password = UserService.get_password_hash(user_create.password)

        # Create the user object
        user = User(
            id=uuid.uuid4(),
            email=user_create.email,
            username=user_create.username,
            password_hash=hashed_password,
            hardware_background=user_create.hardware_background,
            software_background=user_create.software_background
        )

        # Add to database
        db.add(user)
        try:
            await db.commit()
            await db.refresh(user)
        except IntegrityError:
            await db.rollback()
            raise ValueError("Email already registered")

        return UserResponse.from_orm(user)

    @staticmethod
    async def authenticate_user(db: AsyncSession, email: str, password: str) -> Optional[UserResponse]:
        """Authenticate a user by email and password"""
        user = await UserService.get_user_by_email(db, email)
        if not user:
            # To prevent timing attacks, we should still hash a password even if user doesn't exist
            UserService.get_password_hash("dummy_password_for_timing_attack_prevention")
            return None

        # Retrieve user from database to get the password hash
        result = await db.execute(select(User).where(User.email == email))
        db_user = result.scalar_one_or_none()

        if not db_user or not UserService.verify_password(password, db_user.password_hash):
            return None

        return user

    @staticmethod
    async def update_user(db: AsyncSession, user_id: str, update_data: dict) -> Optional[UserResponse]:
        """Update user information"""
        try:
            uuid_obj = uuid.UUID(user_id)
        except ValueError:
            return None

        result = await db.execute(select(User).where(User.id == uuid_obj))
        user = result.scalar_one_or_none()

        if not user:
            return None

        # Update the user with new values
        for field, value in update_data.items():
            if hasattr(user, field):
                if field == "password":
                    # If password is being updated, hash it
                    setattr(user, "password_hash", UserService.get_password_hash(value))
                else:
                    setattr(user, field, value)

        await db.commit()
        await db.refresh(user)

        return UserResponse.from_orm(user)

    @staticmethod
    async def get_current_time() -> datetime:
        """Get current time for token expiration"""
        return datetime.utcnow()