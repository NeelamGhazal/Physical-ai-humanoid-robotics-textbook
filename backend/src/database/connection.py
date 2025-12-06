from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.pool import QueuePool
from backend.src.config import settings
import asyncio

# Create async engine
engine = create_async_engine(
    settings.DATABASE_URL,
    pool_pre_ping=True,
    pool_recycle=300,
    poolclass=QueuePool,
    pool_size=5,
    max_overflow=10,
    echo=True  # Set to False in production
)

# Create session maker
AsyncSessionLocal = sessionmaker(
    autocommit=False,
    autoflush=False,
    bind=engine,
    class_=AsyncSession
)

# Base class for models
Base = declarative_base()


# Dependency to get async database session
async def get_async_db():
    async with AsyncSessionLocal() as session:
        yield session


# Initialize database tables
async def init_db():
    async with engine.begin() as conn:
        # Create all tables
        await conn.run_sync(Base.metadata.create_all)


# Dispose of the engine
async def close_db():
    await engine.dispose()