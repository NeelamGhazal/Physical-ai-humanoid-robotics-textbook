from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from typing import List, Optional
import uuid

from backend.src.database.models import Module, Chapter
from backend.src.schemas.module import ModuleCreate, ModuleUpdate, ModuleResponse
from backend.src.utils.common import generate_uuid


class ModuleService:
    @staticmethod
    async def get_all_modules(db: AsyncSession) -> List[ModuleResponse]:
        """Get all modules from the database"""
        result = await db.execute(select(Module))
        modules = result.scalars().all()

        return [ModuleResponse.from_orm(module) for module in modules]

    @staticmethod
    async def get_module_by_id(db: AsyncSession, module_id: str) -> Optional[ModuleResponse]:
        """Get a specific module by ID"""
        try:
            uuid_obj = uuid.UUID(module_id)
        except ValueError:
            return None

        result = await db.execute(select(Module).where(Module.id == uuid_obj))
        module = result.scalar_one_or_none()

        if module:
            return ModuleResponse.from_orm(module)
        return None

    @staticmethod
    async def create_module(db: AsyncSession, module_create: ModuleCreate) -> ModuleResponse:
        """Create a new module in the database"""
        # Convert the Pydantic model to a database model
        module = Module(
            id=uuid.uuid4(),
            title=module_create.title,
            slug=module_create.slug,
            description=module_create.description,
            module_type=module_create.module_type,
            estimated_duration_weeks=module_create.estimated_duration_weeks,
            learning_objectives=module_create.learning_objectives
        )

        db.add(module)
        await db.commit()
        await db.refresh(module)

        return ModuleResponse.from_orm(module)

    @staticmethod
    async def update_module(db: AsyncSession, module_id: str, module_update: ModuleUpdate) -> Optional[ModuleResponse]:
        """Update an existing module in the database"""
        try:
            uuid_obj = uuid.UUID(module_id)
        except ValueError:
            return None

        result = await db.execute(select(Module).where(Module.id == uuid_obj))
        module = result.scalar_one_or_none()

        if not module:
            return None

        # Update the module with new values
        update_data = module_update.dict(exclude_unset=True)
        for field, value in update_data.items():
            setattr(module, field, value)

        await db.commit()
        await db.refresh(module)

        return ModuleResponse.from_orm(module)

    @staticmethod
    async def delete_module(db: AsyncSession, module_id: str) -> bool:
        """Delete a module from the database"""
        try:
            uuid_obj = uuid.UUID(module_id)
        except ValueError:
            return False

        result = await db.execute(select(Module).where(Module.id == uuid_obj))
        module = result.scalar_one_or_none()

        if not module:
            return False

        await db.delete(module)
        await db.commit()

        return True

    @staticmethod
    async def get_module_chapters(db: AsyncSession, module_id: str) -> List:
        """Get all chapters for a specific module"""
        try:
            uuid_obj = uuid.UUID(module_id)
        except ValueError:
            return []

        result = await db.execute(select(Chapter).where(Chapter.module_id == uuid_obj))
        chapters = result.scalars().all()

        # Import here to avoid circular imports
        from backend.src.schemas.chapter import ChapterResponse
        return [ChapterResponse.from_orm(chapter) for chapter in chapters]