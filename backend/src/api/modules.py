from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from typing import List

from backend.src.database.connection import get_async_db
from backend.src.schemas.module import ModuleResponse, ModuleCreate, ModuleUpdate
from backend.src.schemas.chapter import ChapterResponse
from backend.src.services.modules import ModuleService

router = APIRouter()


@router.get("/", response_model=List[ModuleResponse])
async def get_modules(db: AsyncSession = Depends(get_async_db)):
    """Get all textbook modules"""
    try:
        modules = await ModuleService.get_all_modules(db)
        return modules
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving modules: {str(e)}"
        )


@router.get("/{module_id}", response_model=ModuleResponse)
async def get_module(module_id: str, db: AsyncSession = Depends(get_async_db)):
    """Get a specific module by ID"""
    try:
        module = await ModuleService.get_module_by_id(db, module_id)
        if not module:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Module not found"
            )
        return module
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving module: {str(e)}"
        )


@router.post("/", response_model=ModuleResponse, status_code=status.HTTP_201_CREATED)
async def create_module(module: ModuleCreate, db: AsyncSession = Depends(get_async_db)):
    """Create a new module"""
    try:
        new_module = await ModuleService.create_module(db, module)
        return new_module
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating module: {str(e)}"
        )


@router.put("/{module_id}", response_model=ModuleResponse)
async def update_module(module_id: str, module_update: ModuleUpdate, db: AsyncSession = Depends(get_async_db)):
    """Update an existing module"""
    try:
        updated_module = await ModuleService.update_module(db, module_id, module_update)
        if not updated_module:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Module not found"
            )
        return updated_module
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating module: {str(e)}"
        )


@router.delete("/{module_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_module(module_id: str, db: AsyncSession = Depends(get_async_db)):
    """Delete a module"""
    try:
        success = await ModuleService.delete_module(db, module_id)
        if not success:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Module not found"
            )
        return
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error deleting module: {str(e)}"
        )


@router.get("/{module_id}/chapters", response_model=List[ChapterResponse])
async def get_module_chapters(module_id: str, db: AsyncSession = Depends(get_async_db)):
    """Get all chapters for a specific module"""
    try:
        chapters = await ModuleService.get_module_chapters(db, module_id)
        return chapters
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving module chapters: {str(e)}"
        )