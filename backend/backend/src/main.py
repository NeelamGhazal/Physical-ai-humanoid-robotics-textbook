from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

# Import API routers
from backend.src.api.modules import router as modules_router
from backend.src.api.chapters import router as chapters_router
from backend.src.api.auth import router as auth_router
from backend.src.api.personalization import router as personalization_router
from backend.src.api.chatbot import router as chatbot_router
from backend.src.api.translation import router as translation_router
from backend.src.api.progress import router as progress_router

# Import models for database initialization
from backend.src.database.models import *
from backend.src.database.connection import init_db

app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="API for the Physical AI & Humanoid Robotics textbook platform",
    version="1.0.0",
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routers
app.include_router(modules_router, prefix="/api/v1", tags=["modules"])
app.include_router(chapters_router, prefix="/api/v1", tags=["chapters"])
app.include_router(auth_router, prefix="/api/v1", tags=["auth"])
app.include_router(personalization_router, prefix="/api/v1", tags=["personalization"])
app.include_router(chatbot_router, prefix="/api/v1", tags=["chatbot"])
app.include_router(translation_router, prefix="/api/v1", tags=["translation"])
app.include_router(progress_router, prefix="/api/v1", tags=["progress"])

@app.on_event("startup")
async def startup_event():
    # Initialize database tables
    await init_db()

@app.get("/")
def read_root():
    return {"message": "Physical AI & Humanoid Robotics Textbook API"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=int(os.getenv("PORT", 8000)),
        reload=True
    )