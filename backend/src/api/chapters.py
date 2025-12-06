from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from typing import List

from backend.src.database.connection import get_async_db
from backend.src.schemas.chapter import ChapterResponse, ChapterCreate, ChapterUpdate, ChapterContentResponse
from backend.src.services.chapters import ChapterService

router = APIRouter()


@router.get("/", response_model=List[ChapterResponse])
async def get_chapters(db: AsyncSession = Depends(get_async_db)):
    """Get all textbook chapters"""
    try:
        chapters = await ChapterService.get_all_chapters(db)
        return chapters
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving chapters: {str(e)}"
        )


@router.get("/{chapter_id}", response_model=ChapterResponse)
async def get_chapter(chapter_id: str, db: AsyncSession = Depends(get_async_db)):
    """Get a specific chapter by ID"""
    try:
        chapter = await ChapterService.get_chapter_by_id(db, chapter_id)
        if not chapter:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Chapter not found"
            )
        return chapter
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving chapter: {str(e)}"
        )


@router.get("/{chapter_id}/content", response_model=ChapterContentResponse)
async def get_chapter_content(
    chapter_id: str,
    depth: str = None,
    language: str = "english",
    db: AsyncSession = Depends(get_async_db)
):
    """Get chapter content with optional personalization"""
    try:
        chapter_content = await ChapterService.get_chapter_content(db, chapter_id, depth, language)
        if not chapter_content:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Chapter not found"
            )
        return chapter_content
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving chapter content: {str(e)}"
        )


@router.post("/", response_model=ChapterResponse, status_code=status.HTTP_201_CREATED)
async def create_chapter(chapter: ChapterCreate, db: AsyncSession = Depends(get_async_db)):
    """Create a new chapter"""
    try:
        new_chapter = await ChapterService.create_chapter(db, chapter)
        return new_chapter
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating chapter: {str(e)}"
        )


@router.put("/{chapter_id}", response_model=ChapterResponse)
async def update_chapter(chapter_id: str, chapter_update: ChapterUpdate, db: AsyncSession = Depends(get_async_db)):
    """Update an existing chapter"""
    try:
        updated_chapter = await ChapterService.update_chapter(db, chapter_id, chapter_update)
        if not updated_chapter:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Chapter not found"
            )
        return updated_chapter
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating chapter: {str(e)}"
        )


@router.delete("/{chapter_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_chapter(chapter_id: str, db: AsyncSession = Depends(get_async_db)):
    """Delete a chapter"""
    try:
        success = await ChapterService.delete_chapter(db, chapter_id)
        if not success:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Chapter not found"
            )
        return
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error deleting chapter: {str(e)}"
        )