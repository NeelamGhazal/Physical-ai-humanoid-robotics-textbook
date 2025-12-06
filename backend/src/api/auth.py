from fastapi import APIRouter, Depends, HTTPException, status, Request
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Optional
import jwt

from backend.src.database.connection import get_async_db
from backend.src.schemas.user import UserCreate, UserResponse, UserLogin, TokenResponse
from backend.src.services.users import UserService
from backend.src.config import settings
from backend.src.utils.common import validate_email

router = APIRouter()


@router.post("/register", response_model=UserResponse, status_code=status.HTTP_201_CREATED)
async def register_user(user: UserCreate, db: AsyncSession = Depends(get_async_db)):
    """Register a new user"""
    try:
        # Validate email format
        if not validate_email(user.email):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid email format"
            )

        # Check if user already exists
        existing_user = await UserService.get_user_by_email(db, user.email)
        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="A user with this email already exists"
            )

        # Create the user
        created_user = await UserService.create_user(db, user)
        return created_user
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error registering user: {str(e)}"
        )


@router.post("/login", response_model=TokenResponse)
async def login_user(user_login: UserLogin, db: AsyncSession = Depends(get_async_db)):
    """Authenticate user and return JWT token"""
    try:
        # Verify user credentials
        user = await UserService.authenticate_user(db, user_login.email, user_login.password)
        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Incorrect email or password",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Create JWT token
        token_data = {
            "sub": str(user.id),
            "email": user.email,
            "exp": jwt.timegm((await UserService.get_current_time()).timetuple()) + 3600  # 1 hour expiration
        }
        access_token = jwt.encode(token_data, settings.SECRET_KEY, algorithm=settings.ALGORITHM)

        return TokenResponse(
            access_token=access_token,
            token_type="bearer",
            user=user
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error during login: {str(e)}"
        )


@router.get("/profile", response_model=UserResponse)
async def get_profile(request: Request, db: AsyncSession = Depends(get_async_db)):
    """Get current user's profile information"""
    try:
        # Extract user ID from token (assuming middleware handles authentication)
        user_id = request.state.user_id
        user = await UserService.get_user_by_id(db, user_id)

        if not user:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found"
            )

        return user
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving profile: {str(e)}"
        )


@router.put("/profile", response_model=UserResponse)
async def update_profile(request: Request, user_update: dict, db: AsyncSession = Depends(get_async_db)):
    """Update current user's profile information"""
    try:
        # Extract user ID from token (assuming middleware handles authentication)
        user_id = request.state.user_id

        updated_user = await UserService.update_user(db, user_id, user_update)
        if not updated_user:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found"
            )

        return updated_user
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating profile: {str(e)}"
        )


@router.post("/logout")
async def logout_user():
    """Logout user (client-side token invalidation)"""
    # In a real implementation, you might want to add the token to a blacklist
    return {"message": "Successfully logged out"}