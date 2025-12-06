from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from typing import List, Optional
import uuid

from backend.src.database.models import Chapter, Module
from backend.src.schemas.chapter import ChapterCreate, ChapterUpdate, ChapterResponse, ChapterContentResponse
from backend.src.utils.common import generate_uuid


class ChapterService:
    @staticmethod
    async def get_all_chapters(db: AsyncSession) -> List[ChapterResponse]:
        """Get all chapters from the database"""
        result = await db.execute(select(Chapter))
        chapters = result.scalars().all()

        return [ChapterResponse.from_orm(chapter) for chapter in chapters]

    @staticmethod
    async def get_chapter_by_id(db: AsyncSession, chapter_id: str) -> Optional[ChapterResponse]:
        """Get a specific chapter by ID"""
        try:
            uuid_obj = uuid.UUID(chapter_id)
        except ValueError:
            return None

        result = await db.execute(select(Chapter).where(Chapter.id == uuid_obj))
        chapter = result.scalar_one_or_none()

        if chapter:
            return ChapterResponse.from_orm(chapter)
        return None

    @staticmethod
    async def get_chapter_content(db: AsyncSession, chapter_id: str, depth: str = None, language: str = "english") -> Optional[ChapterContentResponse]:
        """Get chapter content with optional personalization"""
        try:
            uuid_obj = uuid.UUID(chapter_id)
        except ValueError:
            return None

        result = await db.execute(
            select(Chapter, Module)
            .join(Module, Chapter.module_id == Module.id)
            .where(Chapter.id == uuid_obj)
        )
        row = result.first()

        if not row:
            return None

        chapter, module = row

        # Select content based on personalization
        content_to_return = chapter.content
        if depth and chapter.content_levels:
            # If depth is specified and content levels exist, return the appropriate level
            depth_content = chapter.content_levels.get(depth)
            if depth_content:
                content_to_return = depth_content

        # Handle language translation
        if language.lower() == "roman_urdu" and chapter.roman_urdu_content:
            content_to_return = chapter.roman_urdu_content

        # Create response object
        module_response = {
            "id": module.id,
            "title": module.title,
            "slug": module.slug,
            "description": module.description,
            "module_type": module.module_type,
            "estimated_duration_weeks": module.estimated_duration_weeks,
            "learning_objectives": module.learning_objectives,
            "created_at": module.created_at,
            "updated_at": module.updated_at
        }

        return ChapterContentResponse(
            id=chapter.id,
            title=chapter.title,
            content=content_to_return,
            module=module_response,
            learning_objectives=chapter.learning_objectives or [],
            practical_examples=chapter.practical_examples or []
        )

    @staticmethod
    async def create_chapter(db: AsyncSession, chapter_create: ChapterCreate) -> ChapterResponse:
        """Create a new chapter in the database"""
        # Verify that the module exists
        module_result = await db.execute(select(Module).where(Module.id == chapter_create.module_id))
        module = module_result.scalar_one_or_none()
        if not module:
            raise ValueError(f"Module with ID {chapter_create.module_id} does not exist")

        # Create the chapter
        chapter = Chapter(
            id=uuid.uuid4(),
            title=chapter_create.title,
            slug=chapter_create.slug,
            module_id=chapter_create.module_id,
            content=chapter_create.content,
            content_levels=chapter_create.content_levels,
            roman_urdu_content=chapter_create.roman_urdu_content,
            order=chapter_create.order,
            learning_objectives=chapter_create.learning_objectives,
            practical_examples=chapter_create.practical_examples,
            exercises=chapter_create.exercises
        )

        db.add(chapter)
        await db.commit()
        await db.refresh(chapter)

        return ChapterResponse.from_orm(chapter)

    @staticmethod
    async def update_chapter(db: AsyncSession, chapter_id: str, chapter_update: ChapterUpdate) -> Optional[ChapterResponse]:
        """Update an existing chapter in the database"""
        try:
            uuid_obj = uuid.UUID(chapter_id)
        except ValueError:
            return None

        result = await db.execute(select(Chapter).where(Chapter.id == uuid_obj))
        chapter = result.scalar_one_or_none()

        if not chapter:
            return None

        # Update the chapter with new values
        update_data = chapter_update.dict(exclude_unset=True)
        for field, value in update_data.items():
            setattr(chapter, field, value)

        await db.commit()
        await db.refresh(chapter)

        return ChapterResponse.from_orm(chapter)

    @staticmethod
    async def delete_chapter(db: AsyncSession, chapter_id: str) -> bool:
        """Delete a chapter from the database"""
        try:
            uuid_obj = uuid.UUID(chapter_id)
        except ValueError:
            return False

        result = await db.execute(select(Chapter).where(Chapter.id == uuid_obj))
        chapter = result.scalar_one_or_none()

        if not chapter:
            return False

        await db.delete(chapter)
        await db.commit()

        return True